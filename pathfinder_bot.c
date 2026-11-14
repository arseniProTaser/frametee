// Pathfinder Bot -- finds a route (and shortcuts) to the finish with a Go-Explore search over the
// real physics of the active game, then writes it to the timeline as inputs.
//
// Game-agnostic: it only uses the plugin API (clone_world / step_world / world_player_view and the
// input schema by field id: direction, target, jump, fire, hook, weapon). With PF_WITH_DDNET it also
// scans the DDNet map for finish tiles and builds a walking-distance field to steer the search.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <include_cimgui.h>
#include "plugin_api.h"
#include "pf_core.h"

#ifdef PF_WITH_DDNET
#include <ddnet/ddnet_game.h>
#ifndef TILE_FINISH
#define TILE_FINISH 34
#endif
#endif

#define LOG "Pathfinder Bot"

typedef struct {
  const tas_api_t *api;
  const tas_context_t *ctx;
  bool show;
  // settings
  pf_params params;
  bool weapon_mask_ui[6];
  float goal_x, goal_y; // tiles; used when no distance field exists
  bool have_goal;
  int budget_ms;        // CPU time per frame
  // search
  pf_search search;
  bool has_search, running;
  int track, player, player_count, start_tick;
  size_t rec_size;
  unsigned char *rec_buf;
  int f_dir, f_target, f_jump, f_fire, f_hook, f_weapon;
  // distance field (tiles to finish), from the map when the game module offers it
  float *dist;
  int dist_w, dist_h;
  // verification after applying
  int verify_tick, verify_track;
  char status[200];
} bot_t;

// ---------- record <-> action ------------------------------------------------------------

static void action_to_record(const bot_t *b, void *rec, const pf_action *a) {
  const tas_api_t *api = b->api;
  if (b->f_dir >= 0) api->input_set(rec, b->f_dir, a->dir);
  if (b->f_jump >= 0) api->input_set(rec, b->f_jump, a->jump);
  if (b->f_hook >= 0) api->input_set(rec, b->f_hook, a->hook);
  if (b->f_fire >= 0) api->input_set(rec, b->f_fire, a->fire);
  if (b->f_weapon >= 0) api->input_set(rec, b->f_weapon, a->weapon);
  if (b->f_target >= 0) api->input_set_vec2(rec, b->f_target, (ft_vec2){(float)a->aim_x, (float)a->aim_y});
}

// ---------- pf_sim over the plugin API ----------------------------------------------------

static void *sim_clone(void *u, const void *w) { return ((bot_t *)u)->api->clone_world((const ft_world *)w); }
static void sim_copy(void *u, void *d, const void *s) { ((bot_t *)u)->api->copy_world((ft_world *)d, (const ft_world *)s); }
static void sim_destroy(void *u, void *w) { ((bot_t *)u)->api->destroy_world((ft_world *)w); }

static bool sim_step(void *u, void *w, const pf_action *a) {
  bot_t *b = (bot_t *)u;
  for (int i = 0; i < b->player_count; ++i) b->api->input_default(b->rec_buf + (size_t)i * b->rec_size);
  action_to_record(b, b->rec_buf + (size_t)b->player * b->rec_size, a);
  return b->api->step_world((ft_world *)w, b->rec_buf, (uint32_t)b->player_count);
}

static bool sim_view(void *u, const void *w, pf_view *out) {
  bot_t *b = (bot_t *)u;
  ft_player_view v;
  memset(&v, 0, sizeof(v));
  v.struct_size = sizeof(v);
  if (!b->api->world_player_view((const ft_world *)w, b->player, &v) || !(v.flags & FT_PLAYER_ALIVE)) return false;
  *out = (pf_view){v.position.x, v.position.y, v.velocity.x, v.velocity.y, (v.flags & FT_PLAYER_FINISHED) != 0,
                   (v.flags & FT_PLAYER_DISABLED) != 0};
  return true;
}

static float sim_heuristic(void *u, float x, float y) {
  const bot_t *b = (const bot_t *)u;
  if (b->dist) {
    const int tx = (int)floorf(x), ty = (int)floorf(y);
    if (tx < 0 || ty < 0 || tx >= b->dist_w || ty >= b->dist_h) return 1e4f;
    const float d = b->dist[(size_t)ty * (size_t)b->dist_w + (size_t)tx];
    return d < 0.f ? 1e4f : d;
  }
  return fabsf(b->goal_x - x) + fabsf(b->goal_y - y);
}

// ---------- DDNet map: finish tiles + distance field -------------------------------------

#ifdef PF_WITH_DDNET
static bool build_distance_field(bot_t *b) {
  free(b->dist); b->dist = NULL;
  const ft_world *w0 = b->api->get_initial_world();
  const ddnet_collision_t *c = w0 ? dd_world_collision(w0) : NULL;
  if (!c || c->width <= 0 || c->height <= 0) return false;
  const int W = c->width, H = c->height;
  const size_t n = (size_t)W * (size_t)H;
  float *dist = (float *)malloc(n * sizeof(float));
  int *queue = (int *)malloc(n * sizeof(int));
  if (!dist || !queue) { free(dist); free(queue); return false; }
  int qh = 0, qt = 0;
  for (int y = 0; y < H; ++y)
    for (int x = 0; x < W; ++x) {
      const size_t i = (size_t)y * (size_t)W + (size_t)x;
      dist[i] = -1.f;
      if (ddnet_collision_get_tile(c, x, y) == TILE_FINISH || ddnet_collision_get_front_tile(c, x, y) == TILE_FINISH) {
        dist[i] = 0.f; queue[qt++] = (int)i;
      }
    }
  if (qt == 0) { free(dist); free(queue); return false; } // map has no finish tile
  while (qh < qt) {
    const int i = queue[qh++], x = i % W, y = i / W;
    const int nx[4] = {x - 1, x + 1, x, x}, ny[4] = {y, y, y - 1, y + 1};
    for (int k = 0; k < 4; ++k) {
      if (nx[k] < 0 || ny[k] < 0 || nx[k] >= W || ny[k] >= H) continue;
      const size_t j = (size_t)ny[k] * (size_t)W + (size_t)nx[k];
      if (dist[j] >= 0.f) continue;
      if (ddnet_collision_check_point(c, (float)nx[k] * 32.f + 16.f, (float)ny[k] * 32.f + 16.f)) continue; // solid
      dist[j] = dist[i] + 1.f;
      queue[qt++] = (int)j;
    }
  }
  free(queue);
  b->dist = dist; b->dist_w = W; b->dist_h = H;
  return true;
}
#else
static bool build_distance_field(bot_t *b) { free(b->dist); b->dist = NULL; return false; }
#endif

// ---------- start / stop / apply ---------------------------------------------------------

static void bot_stop(bot_t *b) {
  if (b->has_search) pf_free(&b->search);
  b->has_search = b->running = false;
  free(b->rec_buf); b->rec_buf = NULL;
}

static void bot_start(bot_t *b) {
  bot_stop(b);
  const tas_api_t *api = b->api;
  b->track = api->get_selected_track();
  if (b->track < 0) { snprintf(b->status, sizeof(b->status), "Select a track first."); return; }
  int group = 0;
  b->player = api->get_track_player(b->track, &group);
  b->start_tick = api->get_current_tick();
  ft_world *root = api->get_track_world_state_at(b->track, b->start_tick);
  if (b->player < 0 || !root) { snprintf(b->status, sizeof(b->status), "Could not get the world at the playhead."); return; }
  b->player_count = api->world_player_count(root);
  b->rec_size = api->input_record_size();
  b->rec_buf = (unsigned char *)calloc((size_t)b->player_count, b->rec_size);
  b->f_dir = api->input_field_index("direction");
  b->f_target = api->input_field_index("target");
  b->f_jump = api->input_field_index("jump");
  b->f_fire = api->input_field_index("fire");
  b->f_hook = api->input_field_index("hook");
  b->f_weapon = api->input_field_index("weapon");

  const bool field = build_distance_field(b);
  if (!field && !b->have_goal)
    snprintf(b->status, sizeof(b->status), "No finish tile found: running blind. Set a goal for much better results.");

  pf_params p = b->params;
  p.weapon_mask = 0;
  for (int i = 0; i < 6; ++i) if (b->weapon_mask_ui[i]) p.weapon_mask |= 1u << i;
  pf_sim sim = {b, sim_clone, sim_copy, sim_destroy, sim_step, sim_view, (field || b->have_goal) ? sim_heuristic : NULL};
  if (!b->rec_buf || !pf_init(&b->search, sim, p, root)) {
    snprintf(b->status, sizeof(b->status), "Search init failed (tee dead at the playhead?).");
    api->destroy_world(root);
    bot_stop(b);
    return;
  }
  api->destroy_world(root);
  b->has_search = b->running = true;
  b->params.seed = b->params.seed * 1664525u + 1013904223u;
  if (field || b->have_goal) snprintf(b->status, sizeof(b->status), "Searching (%s)...", field ? "map distance field" : "manual goal");
}

static void bot_apply(bot_t *b, int node, bool finished) {
  if (!b->has_search) return;
  pf_action *acts = NULL;
  const int n = pf_extract(&b->search, node, &acts);
  if (n <= 0) { free(acts); snprintf(b->status, sizeof(b->status), "Nothing to apply yet."); return; }
  const tas_api_t *api = b->api;
  int snippet = -1, offset = 0, count = n;
  undo_command_t *create = api->do_create_snippet(b->track, b->start_tick, n, &snippet);
  if (create) api->register_undo_command(create);
  else {
    int available = 0;
    if (!api->find_snippet_at(b->track, b->start_tick, &snippet, &offset, &available) || available <= 0) {
      free(acts); snprintf(b->status, sizeof(b->status), "A snippet in the way: clear the range after the playhead."); return;
    }
    if (count > available) count = available;
  }
  unsigned char *buf = (unsigned char *)calloc((size_t)count, b->rec_size);
  if (!buf) { free(acts); return; }
  if (!api->get_inputs(snippet, offset, count, buf, b->rec_size))
    for (int i = 0; i < count; ++i) api->input_default(buf + (size_t)i * b->rec_size);
  for (int i = 0; i < count; ++i) action_to_record(b, buf + (size_t)i * b->rec_size, &acts[i]);
  undo_command_t *set = api->do_set_inputs(snippet, offset, count, buf, b->rec_size);
  if (set) api->register_undo_command(set);
  free(buf); free(acts);
  if (finished) { b->verify_tick = b->start_tick + count; b->verify_track = b->track; }
  snprintf(b->status, sizeof(b->status), "Wrote %d ticks to the timeline%s.", count, finished ? " (verifying...)" : "");
}

// One frame after applying, ask the editor's own timeline whether the tee really finished.
static void bot_verify(bot_t *b) {
  if (b->verify_tick < 0) return;
  ft_world *w = b->api->get_track_world_state_at(b->verify_track, b->verify_tick);
  const int player = b->api->get_track_player(b->verify_track, NULL);
  if (!w) return;
  ft_player_view v;
  memset(&v, 0, sizeof(v));
  v.struct_size = sizeof(v);
  const bool ok = b->api->world_player_view(w, player, &v) && (v.flags & FT_PLAYER_FINISHED);
  b->api->destroy_world(w);
  if (ok) snprintf(b->status, sizeof(b->status), "VERIFIED in the editor: the tee finishes at tick %d.", b->verify_tick);
  else snprintf(b->status, sizeof(b->status), "MISMATCH: the editor timeline does not finish at tick %d (physics or input mapping differs).", b->verify_tick);
  b->verify_tick = -1;
}

// ---------- UI ----------------------------------------------------------------------------

FT_PLUGIN_ABI_EXPORT()

#ifdef PF_WITH_DDNET
FT_API const char *plugin_game_id(void) { return "ddnet"; }
#endif

FT_API void *plugin_init(tas_context_t *context, const tas_api_t *api) {
  bot_t *b = (bot_t *)calloc(1, sizeof(bot_t));
  if (!b) return NULL;
  b->api = api; b->ctx = context; b->show = true;
  b->params = pf_default_params();
  b->params.seed = 20240607u;
  b->budget_ms = 8;
  b->verify_tick = -1;
  snprintf(b->status, sizeof(b->status), "Select a track, put the playhead at the start, press Start.");
  api->log(FT_LOG_INFO, LOG, "Plugin initialized.");
  return b;
}

FT_API void plugin_update(void *data) {
  bot_t *b = (bot_t *)data;
  if (!b) return;
  igSetCurrentContext(b->ctx->imgui_context);

  bot_verify(b);
  if (b->running && b->has_search) {
    const double t0 = b->api->get_time();
    do pf_run(&b->search, 64); while ((b->api->get_time() - t0) * 1000.0 < (double)b->budget_ms);
  }

  if (igBeginMainMenuBar()) {
    if (igBeginMenu("Pathfinder Bot", true)) { igMenuItem_BoolPtr("Show Window", NULL, &b->show, true); igEndMenu(); }
    igEndMainMenuBar();
  }
  if (!b->show || !b->ctx->ui_visible) return;

  if (igBegin("Pathfinder Bot", &b->show, ImGuiWindowFlags_None)) {
    igTextWrapped("Searches routes with the game's real physics, hook, jumps and (optionally) weapons. "
                  "It can find shortcuts because it never follows a fixed path.");
    igSeparator();
    igSliderInt("CPU per frame (ms)", &b->budget_ms, 1, 30, "%d", ImGuiSliderFlags_None);
    igSliderInt("Max ticks", &b->params.max_ticks, 100, 20000, "%d", ImGuiSliderFlags_None);
    igSliderInt("Max cells (memory)", &b->params.max_cells, 1000, 200000, "%d", ImGuiSliderFlags_None);
    igSliderFloat("Cell size (tiles)", &b->params.cell, 0.1f, 2.0f, "%.2f", ImGuiSliderFlags_None);
    igSliderFloat("Exploration", &b->params.explore, 0.f, 20.f, "%.1f", ImGuiSliderFlags_None);
    static const char *const wn[6] = {"Hammer", "Gun", "Shotgun", "Grenade", "Laser", "Ninja"};
    igText("Allowed weapons (shots can make skips):");
    for (int i = 0; i < 6; ++i) { igCheckbox(wn[i], &b->weapon_mask_ui[i]); if (i < 5) igSameLine(0, -1); }

    igSeparator();
    igCheckbox("Use manual goal (needed if the map has no finish tile)", &b->have_goal);
    if (b->have_goal) {
      igInputFloat("Goal X (tiles)", &b->goal_x, 1.f, 10.f, "%.2f", ImGuiInputTextFlags_None);
      igInputFloat("Goal Y (tiles)", &b->goal_y, 1.f, 10.f, "%.2f", ImGuiInputTextFlags_None);
      if (igButton("Goal = tee position at playhead", (ImVec2){0, 0})) {
        ft_vec2 p;
        const int tr = b->api->get_selected_track();
        if (tr >= 0 && b->api->get_track_position_at(tr, b->api->get_current_tick(), &p)) { b->goal_x = p.x; b->goal_y = p.y; }
      }
      vec2 center = {b->goal_x, b->goal_y};
      vec4 color = {1.f, 0.25f, 0.25f, 0.9f};
      b->api->draw_circle_world(center, 0.6f, color);
    }

    igSeparator();
    if (igButton(b->running ? "Restart" : "Start", (ImVec2){0, 0})) bot_start(b);
    igSameLine(0, -1);
    if (b->has_search && igButton(b->running ? "Pause" : "Resume", (ImVec2){0, 0})) b->running = !b->running;
    igSameLine(0, -1);
    if (b->has_search && igButton("Stop", (ImVec2){0, 0})) bot_stop(b);

    if (b->has_search) {
      const pf_search *s = &b->search;
      igText("Cells: %d / %d   Simulated ticks: %lld", s->ncells, s->p.max_cells, (long long)s->sim_ticks);
      if (s->best_tick >= 0) {
        igText("FINISH found: %d ticks (%.2fs). Keeps improving while running.", s->best_tick, (double)s->best_tick / 50.0);
        if (igButton("Apply best route to timeline", (ImVec2){0, 0})) bot_apply(b, s->best_node, true);
      } else {
        const pf_cell *c = &s->cells[s->closest_cell];
        igText("No finish yet. Closest: %.1f tiles away (tick %d).", c->h, c->tick);
        if (igButton("Apply closest partial route", (ImVec2){0, 0})) bot_apply(b, c->node, false);
      }
    }
    igTextWrapped("%s", b->status);
  }
  igEnd();
}

FT_API void plugin_shutdown(void *data) {
  bot_t *b = (bot_t *)data;
  if (!b) return;
  bot_stop(b);
  free(b->dist);
  free(b);
}

// Tests pf_core.h against a tiny deterministic platformer (gravity, jump, a pit that kills,
// walls, and a hook that pulls toward the first solid tile along the aim).
// gcc -O2 -std=c11 -I.. pf_core_test.c -lm -o pf_core_test && ./pf_core_test
#include <stdio.h>
#include "pf_core.h"

#define W 80
#define H 20
typedef struct { float x, y, vx, vy; int air_jumps; bool prev_jump; bool finished; bool dead; int tick; } mock_t;
static char g_map[H][W + 1];
static float g_goal_x, g_goal_y;

static bool solid(int tx, int ty) { return tx < 0 || tx >= W || ty < 0 || ty >= H ? true : g_map[ty][tx] == '#'; }
static bool grounded(const mock_t *m) { return solid((int)floorf(m->x), (int)floorf(m->y + 0.5f + 0.05f)); }

static void level(int wall_h, int pit_x0, int pit_x1) {
  for (int y = 0; y < H; ++y) { memset(g_map[y], '.', W); g_map[y][W] = 0; }
  for (int x = 0; x < W; ++x) g_map[H - 3][x] = '#';          // floor at y = 17
  for (int x = pit_x0; x < pit_x1; ++x) g_map[H - 3][x] = '.'; // pit (falls out of the map: death)
  for (int y = 0; y < wall_h; ++y) g_map[H - 4 - y][40] = '#'; // wall
  g_goal_x = 70.f; g_goal_y = 16.f;
}
static void init(mock_t *m) { memset(m, 0, sizeof(*m)); m->x = 3.5f; m->y = 16.f; }

static bool step(void *u, void *w, const pf_action *a) {
  (void)u; mock_t *m = (mock_t *)w;
  if (m->dead || m->finished) return true;
  const bool g = grounded(m);
  if (g) m->air_jumps = 1;
  m->vx += (float)a->dir * 0.03f; m->vx *= g ? 0.9f : 0.98f;
  if (a->jump && !m->prev_jump) {
    if (g) m->vy = -0.40f; else if (m->air_jumps > 0) { m->vy = -0.40f; m->air_jumps--; }
  }
  m->prev_jump = a->jump;
  if (a->hook) { // pull toward first solid tile along the aim within 8 tiles
    const float len = sqrtf((float)(a->aim_x * a->aim_x + a->aim_y * a->aim_y));
    if (len > 0) for (float d = 0.5f; d <= 8.f; d += 0.25f) {
      const float hx = m->x + a->aim_x / len * d, hy = m->y + a->aim_y / len * d;
      if (solid((int)floorf(hx), (int)floorf(hy))) { m->vx += (hx - m->x) / d * 0.04f; m->vy += (hy - m->y) / d * 0.04f; break; }
    }
  }
  m->vy += 0.02f;
  float nx = m->x + m->vx, ny = m->y + m->vy;
  if (solid((int)floorf(nx), (int)floorf(m->y))) { nx = m->x; m->vx = 0; }
  if (solid((int)floorf(nx), (int)floorf(ny + 0.5f))) { if (m->vy > 0) ny = floorf(ny + 0.5f) - 0.5f; m->vy = 0; }
  m->x = nx; m->y = ny; m->tick++;
  if (m->y > H + 2) m->dead = true;
  if (m->x >= g_goal_x) m->finished = true;
  return true;
}
static void *clone(void *u, const void *w) { (void)u; mock_t *c = malloc(sizeof(*c)); *c = *(const mock_t *)w; return c; }
static void copy(void *u, void *d, const void *s) { (void)u; *(mock_t *)d = *(const mock_t *)s; }
static void destroy(void *u, void *w) { (void)u; free(w); }
static bool view(void *u, const void *w, pf_view *o) {
  (void)u; const mock_t *m = w; if (m->dead) return false;
  *o = (pf_view){m->x, m->y, m->vx * 50.f, m->vy * 50.f, m->finished, false}; return true;
}
static float heur(void *u, float x, float y) { (void)u; return fabsf(g_goal_x - x) + 0.3f * fabsf(g_goal_y - y); }

static int solve(const char *name, int wall_h, int pit0, int pit1, uint32_t mask, int budget) {
  level(wall_h, pit0, pit1);
  mock_t root; init(&root);
  pf_sim sim = {NULL, clone, copy, destroy, step, view, heur};
  pf_params p = pf_default_params(); p.cell = 0.5f; p.max_cells = 60000; p.max_ticks = 1500; p.weapon_mask = mask;
  pf_search s;
  if (!pf_init(&s, sim, p, &root)) { printf("%s: init failed\n", name); return 1; }
  for (int i = 0; i < budget && s.best_tick < 0; i += 2000) pf_run(&s, 2000);
  if (s.best_tick < 0) { printf("FAIL %s: no finish (cells=%d, closest h=%.1f)\n", name, s.ncells, s.cells[s.closest_cell].h); pf_free(&s); return 1; }
  const int first = s.best_tick;
  pf_run(&s, 200000); // keep optimizing
  pf_action *acts; const int n = pf_extract(&s, s.best_node, &acts);
  // replay from scratch: must reproduce the finish at exactly best_tick
  mock_t r; init(&r); int fin = -1;
  for (int i = 0; i < n; ++i) { step(NULL, &r, &acts[i]); if (r.finished) { fin = i + 1; break; } }
  const int ok = (n == s.best_tick) && (fin == s.best_tick);
  printf("%s %-22s first=%4d best=%4d ticks | replay finish=%4d | cells=%d nodes=%d simticks=%lld\n", ok ? "PASS" : "FAIL", name, first, s.best_tick, fin, s.ncells, s.nnodes, (long long)s.sim_ticks);
  free(acts); pf_free(&s);
  return !ok;
}

int main(void) {
  int bad = 0;
  bad += solve("flat+pit", 0, 20, 24, 0, 400000);
  bad += solve("wall 3", 3, 20, 24, 0, 600000);
  bad += solve("wall 8 (hook needed)", 8, 20, 24, 0, 2000000);
  return bad ? 1 : 0;
}

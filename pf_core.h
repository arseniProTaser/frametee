// pf_core.h -- Go-Explore style route search over a black-box, deterministic simulation.
//
// Why this finds routes AND skips: the search never follows a hand-made path. It keeps an
// archive of reachable situations (position cell + velocity bucket + frozen), always returns
// to promising or under-explored ones, and tries random macro actions (move / jump / hook with
// an aim / weapon shot, held for a few ticks) using the game's real physics. Anything the
// physics allows -- hook swings, rocket/grenade jumps, speed tricks, wall hops -- can therefore
// show up, because nothing in the search rules it out. A distance-to-finish heuristic only
// steers where to explore; it never decides what is legal.
//
// The core knows nothing about DDNet or Frametee: it talks to a pf_sim vtable.
#ifndef PF_CORE_H
#define PF_CORE_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

typedef struct pf_action {
  int8_t dir;                 // -1, 0, 1
  uint8_t jump, hook, fire;   // 0/1
  uint8_t weapon;             // wanted weapon id
  int16_t aim_x, aim_y;       // cursor offset from the tee, pixels
} pf_action;

typedef struct pf_view {
  float x, y;     // tiles
  float vx, vy;   // physics units
  bool finished;
  bool frozen;
} pf_view;

typedef struct pf_sim {
  void *user;
  void *(*clone)(void *user, const void *world);
  void (*copy)(void *user, void *dst, const void *src);
  void (*destroy)(void *user, void *world);
  bool (*step)(void *user, void *world, const pf_action *action); // exactly one tick
  bool (*view)(void *user, const void *world, pf_view *out);       // false = tee dead / gone
  float (*heuristic)(void *user, float x, float y);                // tiles to finish; NULL = none
} pf_sim;

typedef struct pf_params {
  float cell;          // archive cell size in tiles (0.5 = 16px)
  int max_cells;       // memory cap: one world snapshot per cell
  int max_ticks;       // search horizon
  int max_macro;       // longest macro action, ticks
  uint32_t weapon_mask;// bit i set = weapon i may be used (0 = never shoot)
  float explore;       // weight of the "visited little" bonus, in tiles per e-fold
  uint32_t seed;
} pf_params;

typedef struct pf_node { int parent; pf_action a; int n; } pf_node;

typedef struct pf_cell {
  uint64_t key;
  void *world;
  int node, tick;
  float h;
  uint32_t visits;
  pf_action last;
} pf_cell;

typedef struct pf_search {
  pf_sim sim;
  pf_params p;
  pf_cell *cells;
  int ncells;
  int *table, table_cap;
  pf_node *nodes;
  int nnodes, node_cap;
  void *work;
  uint64_t rng;
  int best_tick, best_node;   // fastest finish found, -1 if none
  int closest_cell;           // lowest heuristic seen
  int64_t sim_ticks;
} pf_search;

// ---- helpers ----------------------------------------------------------------

static inline uint64_t pf__mix(uint64_t z) {
  z += 0x9e3779b97f4a7c15ull;
  z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ull;
  z = (z ^ (z >> 27)) * 0x94d049bb133111ebull;
  return z ^ (z >> 31);
}
static inline uint32_t pf__rand(pf_search *s) {
  s->rng ^= s->rng << 13; s->rng ^= s->rng >> 7; s->rng ^= s->rng << 17;
  return (uint32_t)(s->rng >> 16);
}
static inline int pf__vb(float v) {
  const float a = fabsf(v);
  const int b = a < 1.f ? 0 : a < 6.f ? 1 : 2;
  return v < 0 ? -b : b;
}
static inline uint64_t pf__key(const pf_search *s, const pf_view *v) {
  const int64_t cx = (int64_t)floorf(v->x / s->p.cell), cy = (int64_t)floorf(v->y / s->p.cell);
  uint64_t k = pf__mix((uint64_t)cx * 73856093ull ^ (uint64_t)cy * 19349663ull);
  k = pf__mix(k ^ (uint64_t)(pf__vb(v->vx) + 3) * 83492791ull ^ (uint64_t)(pf__vb(v->vy) + 3) * 2654435761ull ^
              (v->frozen ? 0xabcdefull : 0));
  return k ? k : 1;
}
static int pf__lookup(const pf_search *s, uint64_t key) {
  const int mask = s->table_cap - 1;
  for (int i = (int)(key & (uint64_t)mask);; i = (i + 1) & mask) {
    if (!s->table[i]) return -1;
    if (s->cells[s->table[i] - 1].key == key) return s->table[i] - 1;
  }
}
static void pf__table_add(pf_search *s, int cell) {
  const int mask = s->table_cap - 1;
  int i = (int)(s->cells[cell].key & (uint64_t)mask);
  while (s->table[i]) i = (i + 1) & mask;
  s->table[i] = cell + 1;
}
static int pf__new_node(pf_search *s, int parent, const pf_action *a, int n) {
  if (s->nnodes == s->node_cap) {
    s->node_cap = s->node_cap ? s->node_cap * 2 : 4096;
    s->nodes = (pf_node *)realloc(s->nodes, (size_t)s->node_cap * sizeof(pf_node));
    if (!s->nodes) abort();
  }
  s->nodes[s->nnodes] = (pf_node){parent, a ? *a : (pf_action){0}, n};
  return s->nnodes++;
}
static inline float pf__h(const pf_search *s, const pf_view *v) {
  return s->sim.heuristic ? s->sim.heuristic(s->sim.user, v->x, v->y) : 0.f;
}

// ---- lifecycle ----------------------------------------------------------------

static inline pf_params pf_default_params(void) {
  pf_params p = {0};
  p.cell = 0.5f; p.max_cells = 30000; p.max_ticks = 6000; p.max_macro = 8;
  p.weapon_mask = 0; p.explore = 4.0f; p.seed = 12345;
  return p;
}

static bool pf_init(pf_search *s, pf_sim sim, pf_params p, const void *root_world) {
  memset(s, 0, sizeof(*s));
  s->sim = sim; s->p = p;
  if (s->p.cell <= 0.f) s->p.cell = 0.5f;
  if (s->p.max_cells < 16) s->p.max_cells = 16;
  if (s->p.max_macro < 1) s->p.max_macro = 1;
  s->rng = pf__mix(p.seed ? p.seed : 1) | 1;
  s->best_tick = s->best_node = -1;
  s->cells = (pf_cell *)calloc((size_t)s->p.max_cells, sizeof(pf_cell));
  s->table_cap = 1; while (s->table_cap < s->p.max_cells * 2) s->table_cap <<= 1;
  s->table = (int *)calloc((size_t)s->table_cap, sizeof(int));
  s->work = sim.clone(sim.user, root_world);
  if (!s->cells || !s->table || !s->work) return false;
  pf_view v;
  if (!sim.view(sim.user, root_world, &v)) return false;
  pf_cell *c = &s->cells[0];
  c->key = pf__key(s, &v);
  c->world = sim.clone(sim.user, root_world);
  c->node = pf__new_node(s, -1, NULL, 0);
  c->tick = 0; c->h = pf__h(s, &v);
  s->ncells = 1; s->closest_cell = 0;
  pf__table_add(s, 0);
  return c->world != NULL;
}

static void pf_free(pf_search *s) {
  for (int i = 0; i < s->ncells; ++i) if (s->cells[i].world) s->sim.destroy(s->sim.user, s->cells[i].world);
  if (s->work) s->sim.destroy(s->sim.user, s->work);
  free(s->cells); free(s->table); free(s->nodes);
  memset(s, 0, sizeof(*s));
}

// ---- search ---------------------------------------------------------------------

static int pf__pick(pf_search *s) {
  const int limit = s->best_tick >= 0 ? s->best_tick - 1 : s->p.max_ticks;
  const int k = (pf__rand(s) % 10 == 0) ? 1 : 6; // 10%: plain random cell, the rest: tournament
  int best = -1; float best_score = -1e30f;
  for (int i = 0; i < k; ++i) {
    const int idx = (int)(pf__rand(s) % (uint32_t)s->ncells);
    const pf_cell *c = &s->cells[idx];
    if (c->tick >= limit) continue;
    const float score = -c->h - s->p.explore * logf(1.f + (float)c->visits);
    if (score > best_score) { best_score = score; best = idx; }
  }
  return best;
}

// Direction (as an angle in 0..31 steps) in which the heuristic drops fastest from here.
static int pf__downhill(pf_search *s, const pf_view *v) {
  if (!s->sim.heuristic) return (int)(pf__rand(s) % 32);
  int best = 0; float bh = 1e30f;
  for (int i = 0; i < 16; ++i) {
    const float a = (float)i * (6.2831853f / 16.f);
    const float h = s->sim.heuristic(s->sim.user, v->x + cosf(a) * 4.f, v->y + sinf(a) * 4.f);
    if (h < bh) { bh = h; best = i * 2; }
  }
  return best;
}

static void pf__set_aim(pf_action *a, int step32) {
  const float ang = (float)(step32 & 31) * (6.2831853f / 32.f);
  a->aim_x = (int16_t)lroundf(cosf(ang) * 300.f);
  a->aim_y = (int16_t)lroundf(sinf(ang) * 300.f);
}

static void pf__sample(pf_search *s, const pf_cell *c, const pf_view *v, pf_action *a) {
  const uint32_t r = pf__rand(s);
  const bool weapons = s->p.weapon_mask != 0;
  if (r % 100 < 35) { // keep momentum: previous action with one field changed
    *a = c->last;
    switch (pf__rand(s) % 4) {
    case 0: a->dir = (int8_t)((int)(pf__rand(s) % 3) - 1); break;
    case 1: a->jump ^= 1; break;
    case 2: a->hook ^= 1; if (a->hook) pf__set_aim(a, pf__downhill(s, v) + (int)(pf__rand(s) % 7) - 3); break;
    default: pf__set_aim(a, (int)(pf__rand(s) % 32)); break;
    }
    return;
  }
  a->dir = (int8_t)((int)(pf__rand(s) % 3) - 1);
  a->jump = pf__rand(s) % 100 < 30;
  a->hook = pf__rand(s) % 100 < 30;
  a->fire = weapons && pf__rand(s) % 100 < 12;
  a->weapon = 0;
  if (a->fire) { // pick a permitted weapon uniformly
    int ids[32], n = 0;
    for (int i = 0; i < 32; ++i) if (s->p.weapon_mask & (1u << i)) ids[n++] = i;
    if (n) a->weapon = (uint8_t)ids[pf__rand(s) % (uint32_t)n]; else a->fire = 0;
  }
  if (a->hook || a->fire) {
    pf__set_aim(a, (pf__rand(s) % 100 < 45) ? pf__downhill(s, v) + (int)(pf__rand(s) % 7) - 3
                                              : (int)(pf__rand(s) % 32));
  } else { a->aim_x = c->last.aim_x; a->aim_y = c->last.aim_y; }
}

// One expansion: pick a cell, run one random macro action from it, archive what it reaches.
static void pf__expand(pf_search *s) {
  const int ci = pf__pick(s);
  if (ci < 0) return;
  pf_cell *base = &s->cells[ci];
  base->visits++;
  pf_view v0;
  if (!s->sim.view(s->sim.user, base->world, &v0)) return;
  pf_action a = {0};
  pf__sample(s, base, &v0, &a);
  const int dur = 1 + (int)(pf__rand(s) % (uint32_t)s->p.max_macro);
  const int base_tick = base->tick, base_node = base->node;
  s->sim.copy(s->sim.user, s->work, base->world);

  for (int t = 1; t <= dur; ++t) {
    if (!s->sim.step(s->sim.user, s->work, &a)) break;
    s->sim_ticks++;
    pf_view v;
    if (!s->sim.view(s->sim.user, s->work, &v)) break; // died
    const int tick = base_tick + t;
    if (v.finished) {
      if (s->best_tick < 0 || tick < s->best_tick) {
        s->best_node = pf__new_node(s, base_node, &a, t);
        s->best_tick = tick;
      }
      break;
    }
    if (tick >= s->p.max_ticks || (s->best_tick >= 0 && tick >= s->best_tick)) break;

    const uint64_t key = pf__key(s, &v);
    int idx = pf__lookup(s, key);
    if (idx < 0) {
      if (s->ncells >= s->p.max_cells) continue;
      idx = s->ncells++;
      pf_cell *c = &s->cells[idx];
      c->key = key; c->world = s->sim.clone(s->sim.user, s->work);
      c->node = pf__new_node(s, base_node, &a, t);
      c->tick = tick; c->h = pf__h(s, &v); c->last = a;
      if (!c->world) { s->ncells--; continue; }
      pf__table_add(s, idx);
      if (c->h < s->cells[s->closest_cell].h) s->closest_cell = idx;
    } else if (tick < s->cells[idx].tick) { // same situation reached sooner: better
      pf_cell *c = &s->cells[idx];
      s->sim.copy(s->sim.user, c->world, s->work);
      c->node = pf__new_node(s, base_node, &a, t);
      c->tick = tick; c->last = a;
    }
  }
}

static void pf_run(pf_search *s, int iterations) {
  for (int i = 0; i < iterations; ++i) pf__expand(s);
}

// ---- results ----------------------------------------------------------------------

// Per-tick actions leading to `node`. Returns the tick count; *out is malloc'd (caller frees).
static int pf_extract(const pf_search *s, int node, pf_action **out) {
  *out = NULL;
  if (node < 0 || node >= s->nnodes) return 0;
  int total = 0, depth = 0;
  for (int n = node; n >= 0; n = s->nodes[n].parent) { total += s->nodes[n].n; depth++; }
  pf_action *acts = (pf_action *)malloc((size_t)(total ? total : 1) * sizeof(pf_action));
  int *chain = (int *)malloc((size_t)depth * sizeof(int));
  if (!acts || !chain) { free(acts); free(chain); return 0; }
  int d = 0;
  for (int n = node; n >= 0; n = s->nodes[n].parent) chain[d++] = n;
  int pos = 0;
  for (int i = d - 1; i >= 0; --i)
    for (int k = 0; k < s->nodes[chain[i]].n; ++k) acts[pos++] = s->nodes[chain[i]].a;
  free(chain);
  *out = acts;
  return total;
}

#endif // PF_CORE_H

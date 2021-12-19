//
// Yocto/Particle: Demo project for physically-based animation.
//

//
// LICENSE:
//
// Copyright (c) 2020 -- 2020 Fabio Pellacini
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//

#ifndef _YOCTO_PARTICLE_
#define _YOCTO_PARTICLE_

// -----------------------------------------------------------------------------
// INCLUDES
// -----------------------------------------------------------------------------

#include <yocto/yocto_math.h>
#include <yocto/yocto_sampling.h>
#include <yocto/yocto_scene.h>
#include <yocto/yocto_shape.h>

#include <functional>
#include <string>
#include <vector>

// -----------------------------------------------------------------------------
// USING DIRECTIVES
// -----------------------------------------------------------------------------
namespace yocto {

// using directives
using std::function;
using std::string;
using std::vector;

}  // namespace yocto

// -----------------------------------------------------------------------------
// SIMULATION DATA AND API
// -----------------------------------------------------------------------------
namespace yocto {

// Springs
struct particle_spring {
  int   vert0 = -1;
  int   vert1 = -1;
  float rest  = 0;
  float coeff = 0;
};

// Collisions
struct particle_collision {
  int   vert     = 0;
  vec3f position = {0, 0, 0};
  vec3f normal   = {0, 0, 0};
};

// Simulation shape
struct particle_shape {
  // particle data
  vector<vec3f> positions  = {};
  vector<vec3f> normals    = {};
  vector<float> radius     = {};
  vector<float> invmass    = {};
  vector<vec3f> velocities = {};

  // material data
  float spring_coeff = 0;

  // particle emitter
  vec3f     emit_velocity = {0, 0, 0};
  float     emit_rngscale = 0;
  rng_state emit_rng      = {};

  // shape data
  vector<int>   points    = {};
  vector<vec2i> lines     = {};
  vector<vec3i> triangles = {};
  vector<vec4i> quads     = {};

  // simulation data
  vector<vec3f>              old_positions = {};
  vector<vec3f>              forces        = {};
  vector<particle_spring>    springs       = {};
  vector<float>              lambdas       = {};
  vector<particle_collision> collisions    = {};

  // initial configuration to reply animation
  vector<vec3f> initial_positions  = {};
  vector<vec3f> initial_normals    = {};
  vector<vec3f> initial_velocities = {};
  vector<float> initial_invmass    = {};
  vector<float> initial_radius     = {};
  vector<int>   initial_pinned     = {};

  // corresponding shape
  int shape = -1;
};

// Simulation collider
struct particle_collider {
  // Vertex data
  vector<vec3f> positions = {};
  vector<vec3f> normals   = {};
  vector<float> radius    = {};

  // Element data
  vector<vec3i> triangles = {};
  vector<vec4i> quads     = {};

  // bvh
  shape_bvh bvh = {};

  // corresponding shape
  int shape = -1;
};

// Simulation scene
struct particle_scene {
  vector<particle_shape>    shapes    = {};
  vector<particle_collider> colliders = {};
};

// Solver type
enum struct particle_solver_type { mass_spring, position_based };

// Solver names
const auto particle_solver_names = vector<string>{
    "mass_spring", "position_based"};

// Simulation parameters
struct particle_params {
  particle_solver_type solver       = particle_solver_type::mass_spring;
  float                gravity      = 9.8;
  float                deltat       = 0.5 * 1.0 / 60.0;
  int                  mssteps      = 200;
  int                  pdbsteps     = 100;
  int                  frames       = 120;
  float                initvelocity = 0;
  float                dumping      = 2;
  float                minvelocity  = 0.01;
  vec2f                bounce       = {0.05f, 1};
  int                  seed         = 987121;
};

// Initialize the simulation state
void init_simulation(particle_scene& scene, const particle_params& params);

// Simulate one frame
void simulate_frame(particle_scene& scene, const particle_params& params);

// Progress report callback
using progress_callback =
    function<void(const string& message, int current, int total)>;

// Simulate the whole sequence
void simulate_frames(particle_scene& scene, const particle_params& params,
    progress_callback progress_cb);

// Scene creation
particle_scene make_ptscene(
    const scene_data& ioscene, const particle_params& params);

// Scene update
void update_ioscene(scene_data& ioscene, const particle_scene& ptscene);
void flatten_scene(scene_data& ioscene);

// Scene creation
int add_particles(particle_scene& scene, int shape_id,
    const vector<int>& points, const vector<vec3f>& positions,
    const vector<float>& radius, float mass, float random_velocity);
int add_cloth(particle_scene& scene, int shape_id, const vector<vec4i>& quads,
    const vector<vec3f>& positions, const vector<vec3f>& normals,
    const vector<float>& radius, float mass, float coeff,
    const vector<int>& pinned);
int add_collider(particle_scene& scene, int shape_id,
    const vector<vec3i>& triangles, const vector<vec4i>& quads,
    const vector<vec3f>& positions, const vector<vec3f>& normals,
    const vector<float>& radius);

// Get shape properties
void get_positions(const particle_shape& shape, vector<vec3f>& positions);
void get_normals(const particle_shape& shape, vector<vec3f>& normals);

}  // namespace yocto

#endif

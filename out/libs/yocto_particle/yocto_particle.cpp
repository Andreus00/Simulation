//
// Implementation for Yocto/Particle.
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

#include "yocto_particle.h"

#include <yocto/yocto_geometry.h>
#include <yocto/yocto_sampling.h>
#include <yocto/yocto_shape.h>

#include <iostream>
#include <unordered_set>

#define PARTICLE_DIAM 0.01
#define EPSILON 0.0001
#define SOLVER_ITERATIONS 3
// -----------------------------------------------------------------------------
// SIMULATION DATA AND API
// -----------------------------------------------------------------------------
namespace yocto {

particle_scene make_ptscene(
    const scene_data& ioscene, const particle_params& params) {
  // scene
  auto ptscene = particle_scene{};

  // shapes
  static auto velocity = unordered_map<string, float>{
      {"floor", 0}, {"particles", 1}, {"cloth", 0}, {"collider", 0}};
  auto iid = 0;
  for (auto& ioinstance : ioscene.instances) {
    auto& ioshape         = ioscene.shapes[ioinstance.shape];
    auto& iomaterial_name = ioscene.material_names[ioinstance.material];
    if (iomaterial_name == "particles") {
      add_particles(ptscene, iid++, ioshape.points, ioshape.positions,
          ioshape.radius, 1, 1);
    } else if (iomaterial_name == "cloth") {
      auto nverts = (int)ioshape.positions.size();
      add_cloth(ptscene, iid++, ioshape.quads, ioshape.positions,
          ioshape.normals, ioshape.radius, 0.5, 1 / 8000.0,
          {nverts - 1, nverts - (int)sqrt((float)nverts)});
    } else if (iomaterial_name == "collider") {
      add_collider(ptscene, iid++, ioshape.triangles, ioshape.quads,
          ioshape.positions, ioshape.normals, ioshape.radius);
    } else if (iomaterial_name == "floor") {
      add_collider(ptscene, iid++, ioshape.triangles, ioshape.quads,
          ioshape.positions, ioshape.normals, ioshape.radius);
    } else {
      throw std::invalid_argument{"unknown material " + iomaterial_name};
    }
  }

  // done
  return ptscene;
}

void update_ioscene(scene_data& ioscene, const particle_scene& ptscene) {
  for (auto& ptshape : ptscene.shapes) {
    auto& ioshape = ioscene.shapes[ptshape.shape];
    get_positions(ptshape, ioshape.positions);
    get_normals(ptshape, ioshape.normals);
  }
}

void flatten_scene(scene_data& ioscene) {
  for (auto& ioinstance : ioscene.instances) {
    auto& shape = ioscene.shapes[ioinstance.shape];
    for (auto& position : shape.positions)
      position = transform_point(ioinstance.frame, position);
    for (auto& normal : shape.normals)
      normal = transform_normal(ioinstance.frame, normal);
    ioinstance.frame = identity3x4f;
  }
}

// Scene creation
int add_particles(particle_scene& scene, int shape_id,
    const vector<int>& points, const vector<vec3f>& positions,
    const vector<float>& radius, float mass, float random_velocity) {
  auto& shape             = scene.shapes.emplace_back();
  shape.shape             = shape_id;
  shape.points            = points;
  shape.initial_positions = positions;
  shape.initial_normals.assign(shape.positions.size(), {0, 0, 1});
  shape.initial_radius = radius;
  shape.initial_invmass.assign(positions.size(), 1 / (mass * positions.size()));
  shape.initial_velocities.assign(positions.size(), {0, 0, 0});
  shape.emit_rngscale = random_velocity;
  return (int)scene.shapes.size() - 1;
}
int add_cloth(particle_scene& scene, int shape_id, const vector<vec4i>& quads,
    const vector<vec3f>& positions, const vector<vec3f>& normals,
    const vector<float>& radius, float mass, float coeff,
    const vector<int>& pinned) {
  auto& shape             = scene.shapes.emplace_back();
  shape.shape             = shape_id;
  shape.quads             = quads;
  shape.initial_positions = positions;
  shape.initial_normals   = normals;
  shape.initial_radius    = radius;
  shape.initial_invmass.assign(positions.size(), 1 / (mass * positions.size()));
  shape.initial_velocities.assign(positions.size(), {0, 0, 0});
  shape.initial_pinned = pinned;
  shape.spring_coeff   = coeff;
  return (int)scene.shapes.size() - 1;
}
int add_collider(particle_scene& scene, int shape_id,
    const vector<vec3i>& triangles, const vector<vec4i>& quads,
    const vector<vec3f>& positions, const vector<vec3f>& normals,
    const vector<float>& radius) {
  auto& collider     = scene.colliders.emplace_back();
  collider.shape     = shape_id;
  collider.quads     = quads;
  collider.triangles = triangles;
  collider.positions = positions;
  collider.normals   = normals;
  collider.radius    = radius;
  return (int)scene.colliders.size() - 1;
}

// Set shapes
void set_velocities(
    particle_shape& shape, const vec3f& velocity, float random_scale) {
  shape.emit_velocity = velocity;
  shape.emit_rngscale = random_scale;
}

// Get shape properties
void get_positions(const particle_shape& shape, vector<vec3f>& positions) {
  positions = shape.positions;
}
void get_normals(const particle_shape& shape, vector<vec3f>& normals) {
  normals = shape.normals;
}

}  // namespace yocto

// -----------------------------------------------------------------------------
// SIMULATION DATA AND API
// -----------------------------------------------------------------------------
namespace yocto {

// Init simulation
void init_simulation(particle_scene& scene, const particle_params& params) {
  // YOUR CODE GOES HERE
  // init particle status
  int seed = 0;
  for (auto& shape : scene.shapes) {
    shape.emit_rng   = make_rng(params.seed);  //, (seed++) * 2 + 1);
    shape.invmass    = shape.initial_invmass;
    shape.normals    = shape.initial_normals;
    shape.positions  = shape.initial_positions;
    shape.radius     = shape.initial_radius;
    shape.velocities = shape.initial_velocities;
    // initialize forces
    shape.forces.clear();

    for (int i = 0; i < shape.positions.size(); i++)
      shape.forces.push_back(zero3f);

    // initialize pinned particles
    for (auto index : shape.initial_pinned) {
      shape.invmass[index] = 0;
    }

    // initialize velocities
    for (auto& velocity : shape.velocities) {
      velocity += sample_sphere(rand2f(shape.emit_rng)) * shape.emit_rngscale *
                  rand1f(shape.emit_rng);
    }

    // clear springs array
    shape.springs.clear();

    // initialize springs
    if (shape.spring_coeff > 0) {
      // add springs for each edge
      for (auto& edge : get_edges(shape.quads)) {
        shape.springs.push_back({edge.x, edge.y,
            distance(shape.positions[edge.x], shape.positions[edge.y]),
            shape.spring_coeff});
      }
      // add 2 diagonal springs foreach square
      for (auto& quad : shape.quads) {
        shape.springs.push_back({quad.x, quad.z,
            distance(shape.positions[quad.x], shape.positions[quad.z]),
            shape.spring_coeff});
        shape.springs.push_back({quad.y, quad.w,
            distance(shape.positions[quad.y], shape.positions[quad.w]),
            shape.spring_coeff});
      }
    }
  }
  for (auto& collider : scene.colliders) {
    // intiialize bvh
    collider.bvh = {};
    // add quads or triangles to bvh
    if (collider.quads.size() > 0)
      collider.bvh = make_quads_bvh(
          collider.quads, collider.positions, collider.radius);
    else
      collider.bvh = make_triangles_bvh(
          collider.triangles, collider.positions, collider.radius);
  }
}

// check if a point is inside a collider
bool collide_collider(const particle_collider& collider, const vec3f& position,
    vec3f& hit_position, vec3f& hit_normal) {
  // YOUR CODE GOES HERE
  auto               ray = ray3f{position, vec3f{0, 1, 0}};
  shape_intersection isec;

  // call the appropriate intersect method
  if (collider.quads.size() > 0)
    isec = intersect_quads_bvh(
        collider.bvh, collider.quads, collider.positions, ray);
  else
    isec = intersect_triangles_bvh(
        collider.bvh, collider.triangles, collider.positions, ray);

  if (!isec.hit) return false;

  // calculate hit position and normal
  if (collider.quads.size() > 0) {
    auto q       = collider.quads[isec.element];
    hit_position = interpolate_quad(collider.positions[q.x],
        collider.positions[q.y], collider.positions[q.z],
        collider.positions[q.w], isec.uv);
    hit_normal   = normalize(
          interpolate_quad(collider.normals[q.x], collider.normals[q.y],
              collider.normals[q.z], collider.normals[q.w], isec.uv));
  } else {
    auto q       = collider.triangles[isec.element];
    hit_position = interpolate_triangle(collider.positions[q.x],
        collider.positions[q.y], collider.positions[q.z], isec.uv);
    hit_normal   = normalize(interpolate_triangle(collider.normals[q.x],
          collider.normals[q.y], collider.normals[q.z], isec.uv));
  }
  // return the angle between hit hormal and ray direction
  return dot(hit_normal, ray.d) > 0;
}

// simulate mass-spring
void simulate_massspring(particle_scene& scene, const particle_params& params) {
  // YOUR CODE GOES HERE

  // SAVE OLD POSITIONS
  for (auto& particle : scene.shapes) {
    particle.old_positions = particle.positions;
  }

  // COMPUTE DYNAMICS
  for (auto& shape : scene.shapes) {
    for (auto s : range(params.mssteps)) {
      auto particel_number = shape.positions.size();
      auto ddt             = params.deltat / params.mssteps;
      // compute forces
      for (int idx = 0; idx < particel_number; idx++) {
        if (!shape.invmass[idx]) continue;
        shape.forces[idx] = vec3f{0, -params.gravity, 0} / shape.invmass[idx];
      }
      for (auto& spring : shape.springs) {
        auto& p0      = spring.vert0;
        auto& p1      = spring.vert1;
        auto  invmass = shape.invmass[p0] + shape.invmass[p1];
        if (!invmass) continue;
        auto delta_pos  = shape.positions[p1] - shape.positions[p0];
        auto delta_vel  = shape.velocities[p1] - shape.velocities[p0];
        auto spring_dir = normalize(delta_pos);
        auto spring_len = length(delta_pos);
        auto force      = spring_dir * (spring_len / spring.rest - 1) /
                     (spring.coeff * invmass);
        force += dot(delta_vel / spring.rest, spring_dir) * spring_dir /
                 (spring.coeff * 1000 * invmass);
        shape.forces[p0] += force;
        shape.forces[p1] -= force;
      }

      for (int i = 0; i < particel_number; i++) {
        if (!shape.invmass[i]) continue;
        // update velocity and positions using Euler's method
        shape.velocities[i] += ddt * shape.forces[i] * shape.invmass[i];
        shape.positions[i] += ddt * shape.velocities[i];
      }
    }
  }
  // HANDLE COLLISIONS
  for (auto& shape : scene.shapes) {
    for (int k = 0; k < shape.positions.size(); k++) {
      if (!shape.invmass[k]) continue;
      for (auto& collider : scene.colliders) {
        auto hitpos = zero3f, hit_normal = zero3f;
        if (collide_collider(
                collider, shape.positions[k], hitpos, hit_normal)) {
          shape.positions[k] = hitpos + hit_normal * 0.005f;
          auto projection    = dot(shape.velocities[k], hit_normal);
          shape.velocities[k] =
              (shape.velocities[k] - projection * hit_normal) *
                  (1.f - params.bounce.x) -
              projection * hit_normal * (1.f - params.bounce.y);
        }
      }
    }
  }
  // VELOCITY FILTER
  for (auto& shape : scene.shapes) {
    for (int k = 0; k < shape.positions.size(); k++) {
      if (!shape.invmass[k]) continue;
      shape.velocities[k] *= (1.f - params.dumping * params.deltat);
      if (length(shape.velocities[k]) < params.minvelocity)
        shape.velocities[k] = {0, 0, 0};
    }
  }
  // RECOMPUTE NORMALS
  for (auto& shape : scene.shapes) {
    if (shape.quads.size() > 0)
      shape.normals = quads_normals(shape.quads, shape.positions);
    else
      shape.normals = triangles_normals(shape.triangles, shape.positions);
  }
}

// simulate pbd
void simulate_pbd(particle_scene& scene, const particle_params& params) {
  // // YOUR CODE GOES HERE

  // SAVE OLD POSITOINS
  for (auto& particle : scene.shapes) {
    particle.old_positions = particle.positions;
  }

  for (auto& shape : scene.shapes) {
    // PREDICT POSITIONS
    for (int i = 0; i < shape.positions.size(); i++) {
      if (!shape.invmass[i]) continue;
      // apply semi-implicit Euler to external forces
      shape.velocities[i] += vec3f{0, -params.gravity, 0} * params.deltat;
      shape.positions[i] += shape.velocities[i] * params.deltat;
    }
    // COMPUTE COLLISIONS
    shape.collisions.clear();
    for (int i = 0; i < shape.positions.size(); i++) {
      if (!shape.invmass[i]) continue;
      for (auto& collider : scene.colliders) {
        auto hit_position = zero3f, hit_normal = zero3f;
        if (!collide_collider(
                collider, shape.positions[i], hit_position, hit_normal))
          continue;
        shape.collisions.push_back({i, hit_position, hit_normal});
      }
    }
    // SOLVE CONSTRAINTS
    for (auto i : range(params.pdbsteps)) {
      // solve springs
      for (auto& spring : shape.springs) {
        auto p0      = spring.vert0;
        auto p1      = spring.vert1;
        auto invmass = shape.invmass[p0] + shape.invmass[p1];
        if (!invmass) continue;
        auto dir = shape.positions[p1] - shape.positions[p0];
        auto len = length(dir);
        dir /= len;
        auto lambda = (1 - spring.coeff) * (len - spring.rest) / invmass;
        shape.positions[p0] += shape.invmass[p0] * lambda * dir;
        shape.positions[p1] -= shape.invmass[p1] * lambda * dir;
      }
      // solve collisions
      for (auto& collision : shape.collisions) {
        auto& particle = shape.positions[collision.vert];
        if (!shape.invmass[collision.vert]) continue;
        auto projection = dot(particle - collision.position, collision.normal);
        if (projection >= 0) continue;
        particle += -projection * collision.normal;
      }
      // solve other collisions
    }
    // COMPUTE VELOCITIES
    for (int i = 0; i < shape.positions.size(); i++) {
      if (!shape.invmass[i]) continue;
      shape.velocities[i] = (shape.positions[i] - shape.old_positions[i]) /
                            params.deltat;
    }
    // VELOCITY FILTER
    for (int i = 0; i < shape.positions.size(); i++) {
      if (!shape.invmass[i]) continue;
      // damping
      shape.velocities[i] *= (1 - params.dumping * params.deltat);
      // sleeping
      if (length(shape.velocities[i]) < params.minvelocity)
        shape.velocities[i] = {0, 0, 0};
    }
    // RECOMPUTE NORMALS
    if (shape.quads.size() > 0)
      shape.normals = quads_normals(shape.quads, shape.positions);
    else
      shape.normals = triangles_normals(shape.triangles, shape.positions);
  }
}

// Simulate one step
void simulate_frame(particle_scene& scene, const particle_params& params) {
  switch (params.solver) {
    case particle_solver_type::mass_spring:
      return simulate_massspring(scene, params);
    case particle_solver_type::position_based:
      return simulate_pbd(scene, params);
    default: throw std::invalid_argument("unknown solver");
  }
}

// Simulate the whole sequence
void simulate_frames(particle_scene& scene, const particle_params& params,
    progress_callback progress_cb) {
  // handle progress
  auto progress = vec2i{0, 1 + (int)params.frames};

  if (progress_cb) progress_cb("init simulation", progress.x++, progress.y);
  init_simulation(scene, params);

  for (auto idx = 0; idx < params.frames; idx++) {
    if (progress_cb) progress_cb("simulate frames", progress.x++, progress.y);
    simulate_frame(scene, params);
  }

  if (progress_cb) progress_cb("simulate frames", progress.x++, progress.y);
}

////////////////////////////////

// simulate pbd
void simulate_pbd_2(particle_scene& scene, const particle_params& params) {
  // YOUR CODE GOES HERE

  // add all rigid body shape constraints
  for (int i = 0; i < scene.shapes.size(); i++) {
  }
  // for all the particles
  for (auto& shape : scene.shapes) {
    for (int i = 0; i < shape.positions.size(); i++) {
      // apply forces
      if (shape.invmass[i] == 0) continue;
      shape.velocities[i] += params.deltat * vec3f{0, -params.gravity, 0} +
                             params.deltat * shape.forces[i];
      shape.positions[i] += params.deltat * shape.velocities[i];
      // mass scale
      // if (shape.invmass[i] == 0 != 0.0) {
      //   shape.invmass[i] = 1.f / ((1.f / shape.invmass[i]) *
      //   exp(-shape.positions[]));
      // } else {
      //   shape.invmass[i] == 0 = 0.0;
      // }
    }
  }
  // for all the particles
  for (int x = 0; x < scene.shapes.size(); x++) {
    auto shape = scene.shapes[x];
    for (int i = 0; i < shape.positions.size(); i++) {
      // find neighboring particles and solid contacts
      for (int y = 0; y < scene.shapes.size(); y++) {
        if (x == y) continue;
        auto shape2 = scene.shapes[y];
        for (int j = 0; j < shape2.positions.size(); j++) {
          if (shape.invmass[i] == 0 && shape2.invmass[j] == 0) continue;
          auto dist = distance(shape.positions[i], shape2.positions[j]);
          if (dist < PARTICLE_DIAM - EPSILON) }
      }
    }
  }

  // Simulate one step
  void simulate_frame(particle_scene & scene, const particle_params& params) {
    switch (params.solver) {
      case particle_solver_type::mass_spring:
        return simulate_massspring(scene, params);
      case particle_solver_type::position_based:
        return simulate_pbd(scene, params);
      default: throw std::invalid_argument("unknown solver");
    }
  }

  // Simulate the whole sequence
  void simulate_frames(particle_scene & scene, const particle_params& params,
      progress_callback progress_cb) {
    // handle progress
    auto progress = vec2i{0, 1 + (int)params.frames};

    if (progress_cb) progress_cb("init simulation", progress.x++, progress.y);
    init_simulation(scene, params);

    for (auto idx = 0; idx < params.frames; idx++) {
      if (progress_cb) progress_cb("simulate frames", progress.x++, progress.y);
      simulate_frame(scene, params);
    }

    if (progress_cb) progress_cb("simulate frames", progress.x++, progress.y);
  }

}  // namespace yocto

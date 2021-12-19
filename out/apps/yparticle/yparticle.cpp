//
// LICENSE:
//
// Copyright (c) 2016 -- 2021 Fabio Pellacini
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include <yocto/yocto_cli.h>
#include <yocto/yocto_math.h>
#include <yocto/yocto_scene.h>
#include <yocto/yocto_sceneio.h>
#include <yocto/yocto_shape.h>
#include <yocto/yocto_trace.h>
#include <yocto_particle/yocto_particle.h>
#if YOCTO_OPENGL
#include <yocto_gui/yocto_glview.h>
#endif
using namespace yocto;

void run_offline(const string& filename, const string& output,
    const particle_params& params) {
  // loading scene
  print_progress_begin("load scene");
  auto error = string{};
  auto scene = scene_data{};
  if (!load_scene(filename, scene, error)) print_fatal(error);
  print_progress_end();

  // flatten scene
  print_progress_begin("flatten scene");
  flatten_scene(scene);
  print_progress_end();

  // initialize particles
  print_progress_begin("make particles");
  auto ptscene = make_ptscene(scene, params);
  print_progress_end();

  // simulation state
  print_progress_begin("simulate particles", params.frames);
  init_simulation(ptscene, params);
  for (auto frame = 0; frame < params.frames; frame++) {
    simulate_frame(ptscene, params);
    print_progress_next();
  }

  // update scene
  update_ioscene(scene, ptscene);

  // render
  auto trparams       = trace_params{};
  trparams.samples    = 16;
  trparams.resolution = 720;
  trparams.sampler    = trace_sampler_type::eyelight;
  print_progress_begin("render image");
  auto image = trace_image(scene, trparams);
  print_progress_end();

  // save
  print_progress_begin("save image");
  if (!save_image(output, image, error)) print_fatal(error);
  print_progress_end();
}

void run_interactive(const string& filename, const string& output,
    const particle_params& params) {
  // loading scene
  print_progress_begin("load scene");
  auto error = string{};
  auto scene = scene_data{};
  if (!load_scene(filename, scene, error)) print_fatal(error);
  print_progress_end();

  // flatten scene
  print_progress_begin("flatten scene");
  flatten_scene(scene);
  print_progress_end();

  // initialize particles
  print_progress_begin("make particles");
  auto ptscene = make_ptscene(scene, params);
  print_progress_end();

  // simulation state
  auto frame = 0;

  // run viewer
  glview_scene(
      "yparticle", filename, scene, {},
      [&](const glinput_state& input, vector<int>&, vector<int>&) {
        if (begin_glheader("simulation")) {
          draw_glprogressbar("frame", frame, params.frames);
          end_glheader();
        }
      },
      [](const glinput_state& input, vector<int>&, vector<int>&) {},
      [&](const glinput_state& input, vector<int>& updated_shapes,
          vector<int>&) {
        if (frame > params.frames) frame = 0;
        if (frame == 0) init_simulation(ptscene, params);
        simulate_frame(ptscene, params);
        frame++;
        update_ioscene(scene, ptscene);
        for (auto& ptshape : ptscene.shapes) {
          updated_shapes.push_back(ptshape.shape);
        }
      });
}

// run
void run(const vector<string>& args) {
  // params
  auto params      = particle_params{};
  auto filename    = "scene.json"s;
  auto output      = "output.png"s;
  auto interactive = false;

  // parse cli
  auto error = string{};
  auto cli   = make_cli("yparticle", "Simulate particles");
  add_option(cli, "scene", filename, "Input scene.");
  add_option(cli, "output", output, "Output image.");
  add_option(cli, "interactive", interactive, "Run interactively.");
  add_option(cli, "frames", params.frames, "Frames");
  add_option(cli, "solver", params.solver, "Solver", particle_solver_names);
  add_option(cli, "gravity", params.gravity, "Gravity");
  if (!parse_cli(cli, args, error)) return print_fatal(error);

  // run
  if (!interactive) {
    run_offline(filename, output, params);
  } else {
    run_interactive(filename, output, params);
  }
}

int main(int argc, const char* argv[]) {
  handle_errors(run, make_cli_args(argc, argv));
}

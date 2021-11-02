#include "geometrycentral/surface/direction_fields.h"

#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>
#include <polyscope/point_cloud.h>

#include "args/args.hxx"
#include "imgui.h"
#include "app.hpp"

#include <vector>
#include <array>


int main(int argc, char **argv) {
  App app = App();
  app.run();

  return EXIT_SUCCESS;
}
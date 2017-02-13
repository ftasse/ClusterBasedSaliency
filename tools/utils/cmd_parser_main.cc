#include <stdio.h>

#include "cmd_parser.h"

int main(int argc, char** argv) {
  using namespace std;

  std::map<string, string> flags;
  std::vector<string> arguments;

  parseCmdArgs(argc, argv, &flags, &arguments);
  
  int num_steps;
  getFlag(flags, "num_steps", 0, &num_steps);
  printf("num_steps: %d\n", num_steps);
  
  return 0;
}

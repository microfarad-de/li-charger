/*
 * Lithium Battery Charger
 * 
 * Author:  Karim Hraibi
 * Version: 0.1.0
 * Date:    13.12.2018
 */
#define VERSION_MAJOR 0  // major version
#define VERSION_MINOR 1  // minor version
#define VERSION_MAINT 0  // maintenance version


#include "cli.h"

uint32_t counter = 0;


void setup (void) {

  Cli.init();

  Cli.xputs ("");
  Cli.xputs ("+ + +  L I T H I U M  C H A R G E R  + + +");
  Cli.xputs ("");
  Cli.xprintf ("Version: %d.%d.%d\n", VERSION_MAJOR, VERSION_MINOR, VERSION_MAINT);
  Cli.xputs ("");

  Cli.newCmd ("hello", "Prints a hello message", cmdHello);
  Cli.newCmd ("e", "", cmdHello);
  Cli.newCmd ("count", "Show the counter", cmdCount);
  Cli.newCmd ("c", "", cmdCount);
  Cli.showHelp ();

}

void loop (void) {

  Cli.getCmd ();

  counter++;
}


int cmdHello (int argc, char **argv) {
  int i;
  Cli.xputs ("Hello");
  for (i = 0; i < argc; i++) {
    Cli.xprintf ("%d: %s", i, argv[i]);
    Cli.xputs ("");
  }
      
  return 0;
}


int cmdCount (int argc, char **argv) {
  int i;
  Cli.xprintf ("Counter = %lu\n", counter);
      
  return 0;
}

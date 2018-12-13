/*
 * Command-line interpreter
 * 
 * Karim Hraibi - 2018
 *
 */

#ifndef __Cli_H_
#define __Cli_H_


#define CLI_NUM_CMD    10     // Number of cli commands
#define CLI_NUM_ARG    5      // Number of cli arguments
#define CLI_ARG_LEN    8      // Maximum argument length



/* 
 * CLI command structure 
 */
typedef struct 
{
  const char *str;          // Command name
  const char *doc;          // Command description
  int (*fct)(int, char **); // Pointer to command function
} Cli_Cmd_s;


/* 
 * Command line interpreter class
 */
class CliClass
{
  public:
    /* 
     * Initialize a CLI object
     */
    void init (void);

    /* 
     * Define a new CLI command
     */
    int newCmd (
      const char *name,             // Command name
      const char *description,      // Command description
      int (*function)(int, char **) // Pointer to a function to be executed by this command
      );

    /*
     * Prompt user for a command - non-blocking function
     * Return: EXIT_FAILURE or the return value of the command function
     *
     * If user enters "h" or "help" a help screen with a list of commands and
     * their descriptions will be displayed.
     *
     * If two commands point to the same function, only the first one will be
     * displayed in the help screen.
     */
    int getCmd (void);

    /* 
     * Print a list of available commands and their description
     */
    void showHelp (void);
    
    /* 
     * Emulate stdio functions
     */
    void xprintf (const char *fmt, ... );
    void xputs (const char *c);
    void xputchar (int c);
    int xgetchar (void);

    
  private:  
    
    void textPadding (char c, int size);                              // Insert repeated sequence of characters
    void textPrintBlock (const char *text, int lineSize, int offset); // Print a formatted block of text
    void sortCmds (int numCmds, Cli_Cmd_s **cmd);                     // Sort commands in alphabetical order
 
    char argBuf[CLI_NUM_ARG][CLI_ARG_LEN]; // Array of charactars for commands and arguments
    char *argv[CLI_NUM_ARG];               // Array of pointers to argument strings
    Cli_Cmd_s cmd[CLI_NUM_CMD];            // Array of commands
    int numCmds;                           // Total number of commands
    bool initialized = false;              // Initialization status

    // The following parameters are used by getCmd ()
    enum {START, DISCARD_LEADING_SPACES, CAPTURE_STRING, DISCARD_TRAILING_SPACES, EVALUATE} state = START; // State
    int argc = 0;                          // Number of arguments
    int idx = 0;                           // Character index
    
};


/*
 * CLI object as as singleton
 */
extern CliClass Cli;


#endif /* __Cli_H_ */

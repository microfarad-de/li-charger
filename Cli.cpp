/* 
 * Command-Line Interpreter
 * 
 * This source file is part of the Lithium-Ion Battery Charger Arduino firmware
 * found under http://www.github.com/microfarad-de/li-charger
 * 
 * Please visit:
 *   http://www.microfarad.de
 *   http://www.github.com/microfarad-de
 * 
 * Copyright (C) 2019 Karim Hraibi (khraibi at gmail.com)
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 */
 
#include <Arduino.h>
//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>
//#include <stdarg.h>
#include "Cli.h"


#define TEXT_LINE_SIZE  70        // Sets the maximum text line size
#define INDENT          15        // Help text block indentation



CliClass Cli;



void CliClass::init (uint32_t serialBaud)
{
  int i;
  this->numCmds = 0;
  this->initialized = true;
  
  for (i = 0; i < CLI_NUM_ARG; i++) {
    this->argv[i] = this->argBuf[i];
  }
  
  Serial.begin (serialBaud);
}


int CliClass::newCmd (const char *name, const char *description, int(*function)(int, char **))
{
  if (!initialized) return EXIT_FAILURE;
  
  if (this->numCmds >= CLI_NUM_CMD) {
    xprintf("OVF!\n");
    return EXIT_FAILURE;
  }

  this->cmd[this->numCmds].str = name;
  this->cmd[this->numCmds].doc = description;
  this->cmd[this->numCmds].fct = function;
  this->numCmds++;

  return EXIT_SUCCESS;
}


int CliClass::getCmd(void)
{
  char c;
  int i;
  int rv;

  c = (char)(xgetchar() & 0x000000FF);

  switch (state) {

    case START:
      argc = 0;
      idx = 0;
      for (i = argc; i < CLI_NUM_ARG; i++) {
        argv[i][0] = 0;
      }
      state = DISCARD_LEADING_SPACES;
      break;

    case DISCARD_LEADING_SPACES:
      if ((c == ' ') || (c == '\t') || (c == '\r') || (c == '\n')) {
        return EXIT_FAILURE;
      }
      else if (c > ' ') {
        argv[argc][idx] = c;
        idx++;
        state = CAPTURE_STRING;
      }
      break;
        
    case CAPTURE_STRING:
      if ((c == ' ') || (c == '\t') || (idx >= CLI_ARG_LEN - 1)) {
        argv[argc][idx] = 0;
        argc++;
        idx = 0;
        state = DISCARD_TRAILING_SPACES;
      }
      else if ((c == '\r') || (c == '\n')) {
        argv[argc][idx] = 0;
        argc++;
        idx = 0;
        state = EVALUATE;
      }
      else if (c > ' ') {
        argv[argc][idx] = c;
        idx++;
      }
      break;

    case DISCARD_TRAILING_SPACES:
      if ((c == ' ') || (c == '\t')) {
        return EXIT_FAILURE;
      }
      else if ((c == '\r') || (c == '\n')) {
        state = EVALUATE;
      }
      else if ((c > ' ') && (argc < CLI_NUM_ARG)) {
        argv[argc][idx] = c;
        idx++;
        state = CAPTURE_STRING;    
      }
      break;

    case EVALUATE:
      state = START;
      if (strcmp(argv[0], "h") == 0) {
        showHelp();
        return EXIT_SUCCESS;
      }
      for (i = 0; i < this->numCmds; i++) {
        if (strcmp(argv[0], this->cmd[i].str) == 0) {
          rv = this->cmd[i].fct(argc, argv);
          return rv;
        }
      }        
      xprintf("Unknown '%s'\n", argv[0]);
      xputs("");
      break;

    default:       
      break;      
  }

  return EXIT_FAILURE;
}


void CliClass::showHelp (void)
{
  int i, j;
  bool duplicate = false;
  CliCmd_s *cmd[CLI_NUM_CMD];
  int numCmds = 0;
  int len = 0;

  if (!initialized) return EXIT_FAILURE;

  for (i = 0; i < this->numCmds && (i < CLI_NUM_CMD); i++) {

      // Search for duplicate commands
      duplicate = false;
      for (j = 0; j < i; j++) {
          if (this->cmd[i].fct == this->cmd[j].fct)
              duplicate = true;
      }
      // Do not show the same command twice
      if (!duplicate) {
          cmd[numCmds] = &this->cmd[i];
          numCmds++;
      }
  }

  // Sort commands in alphabetical order
  sortCmds(numCmds, cmd);

  xprintf("Commands:\n");

  // Display commands
  for (i = 0; (i < numCmds) && (i < CLI_NUM_CMD); i++) {

    xprintf("  %s", cmd[i]->str);
    len = strlen(cmd[i]->str) + 2;

    // Search for alternative commands and display them between parantheses
    duplicate = false;
    for (j = 0; (j < this->numCmds) && (j < CLI_NUM_CMD); j++) {
      if ((this->cmd[j].fct == cmd[i]->fct) && (strcmp(this->cmd[j].str, cmd[i]->str) != 0)) {
        if (!duplicate) {
          xprintf(" (");
          len += 2;
        }
        else {
          xprintf(", ");
          len += 2;
        }
        xprintf(this->cmd[j].str);
        len += strlen(this->cmd[j].str);
        duplicate = true;
      }
    }
    
    if (duplicate) {
      xprintf(")");
      len += 1;
    }
    
    textPadding(' ', INDENT - len - 2);
    xprintf(": ");
    textPrintBlock(cmd[i]->doc, TEXT_LINE_SIZE, INDENT);
  }
  
  xprintf("  h");
  textPadding(' ', INDENT - 3 - 2);
  xprintf(": ");
  textPrintBlock("help screen", TEXT_LINE_SIZE, INDENT);
  xputs("");
}


void CliClass::sortCmds (int numCmds, CliCmd_s **cmd)
{
  bool sorted = false;
  int i;
  CliCmd_s *temp;

  while (!sorted) {
    sorted = true;
    for (i = 0; i < numCmds - 1; i++) {
      if (strcmp(cmd[i]->str, cmd[i + 1]->str) > 0) {
        sorted = false;
        temp = cmd[i];
        cmd[i] = cmd[i + 1];
        cmd[i + 1] = temp;
      }
    }
  }
}


void CliClass::textPrintBlock (const char *text, int lineSize, int offset)
{
  const char *c = text;
  int count, i;

  while (*c != 0) {
    // remove leading spaces
    while (*c == ' ' || *c == '\t') c++;

    count = 0;
    // write a line
    while (count < (lineSize - offset) && *c != '\r' && *c != '\n' && *c != 0) {
      xputchar(*c);
      c++;
      count++;
    }

    // finish the last word of a line
    while (*c != ' ' && *c != '\t' && *c != '\r' && *c != '\n' && *c != 0) {
      xputchar(*c);
      c++;
    }

    // remove trailing spaces
    while (*c == ' ' || *c == '\t') c++;

    // remove one line-break
    if (*c == '\r' || *c == '\n') c++;

    // add new-line and padding
    if (*c != 0) {
      xputchar('\n');
      for (i = 0; i < offset; i++) xputchar(' ');
    }
  }
  xputchar('\n');
}


void CliClass::textPadding (char c, int size)
{
  int i;

  for (i = 0; i < size; i++) {
    xputchar(c);
  }
}


void CliClass::xprintf (const char *fmt, ... ){
  va_list args;
  if (!initialized) return;
  va_start (args, fmt );
  vsnprintf(printfBuf, CLI_PRINTF_BUF_SIZE, fmt, args);
  va_end (args);
  Serial.print(printfBuf);
}


void CliClass::xputs (const char *c){
  if (!initialized) return;
  Serial.print(c);
  Serial.print("\n");
}


void CliClass::xputchar (int c){
  if (!initialized) return;
  Serial.write(c);
}


int CliClass::xgetchar (void){
  int c;
  if (!initialized) return -1;
  c = Serial.read();
  return c;
}

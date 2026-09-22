#pragma once
#include "globals.h"

void kbAppend(const String& ch);
void kbAppendCp(uint16_t cp);
void kbBackspace();
void handleKeyboardTap(int x, int y);
void handleNetTap(int x, int y, int dx, int dy);
void pollTouch();
#pragma once
#include <stdio.h>
#include <tchar.h>
#include <conio.h>
#include "control.cpp"
// RTC5 header file for implicitly linking to the RTC5DLL.DLL
#include "RTC5impl.h"
//#include "clipper2.h"

static const int m_stepRatio = global_config.value("m_stepRatio", 5882);;


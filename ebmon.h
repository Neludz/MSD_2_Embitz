/**
 * @file
 * @brief
 *
 * @section License
 *
 * Copyright (C) 2021 All rights reserved.
 *
 *
 * @author Gerard Zagema
 * @version 1.0.0
 **/

#ifndef __EBMON_H__
#define __EBMON_H__

#include <stdint.h>
#include <stdio.h>


/** \brief Flush (clear) a pipe buffer */
void EBmonitor_flush(FILE* );

/** \brief Check if there is any data in the stdin pipe */
int  EBmonitor_kbhit();

#endif

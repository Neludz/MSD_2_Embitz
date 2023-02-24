/**
 * @file EmBitz EBmonitor target
 * @brief This source file must be build and linked in order to enable EBmonitor
 *
 * @section License
 *
 * Copyright (C) 2021 All rights reserved.
 *
 *
 * @author Gerard Zagema
 * @version 1.3.0
 **/

/** \defgroup EmBitz EBmonitor debug tool
  @{
 */

// REMARK: Don't change the names otherwise EmBitz can not find them!!!

#include "ebmon.h"

/**\brief Uncomment this #define to skip EBmonitor initialize function */
//#define NO_EBMON_INIT

/**\brief Initial buffer sizes */
#ifndef STDOUT_BUFFERSIZE
#define STDOUT_BUFFERSIZE 256
#endif

#ifndef STDIN_BUFFERSIZE
#define STDIN_BUFFERSIZE  64
#endif

/**\brief The actual buffers to hold the data of the STDIO pipes */
static char _stdout_buffer[STDOUT_BUFFERSIZE] __attribute__((aligned (4)));
static char _stdin_buffer[STDIN_BUFFERSIZE]   __attribute__((aligned (4)));

/**\brief The definition of a pipe control block  */
typedef struct _std_pipe
{
    uint16_t length __attribute__((aligned (4)));         // Length of the pipe buffer
    volatile uint16_t tail __attribute__((aligned (4)));  // Tail of the circular buffer
    volatile uint16_t head __attribute__((aligned (4)));  // Head of the circular buffer
    void *ptr __attribute__((aligned (4)));               // Pointer to the actual buffer, keep as void* !!!
} _std_pipe __attribute__((aligned (4)));


/**\brief  Our STDIO pipes */
static _std_pipe _eb_monitor_stdout = {STDOUT_BUFFERSIZE,0,0, _stdout_buffer};
static _std_pipe _eb_monitor_stdin  = {STDIN_BUFFERSIZE, 0,0, _stdin_buffer };


#ifndef NO_EBMON_INIT
/** \brief If additional initialization is needed then this is the place.
 *         The initialization is done before the first char is written to STDOUT pipe.
 *
 *
 */
static uint8_t _eb_initialized = 0;
static void ebmonitor_init (void)
{
    // Start with a clean screen when start writing.
    _stdout_buffer[1] = '\f';
    _eb_monitor_stdout.head = 1;
}
#endif



/** \brief Flush (clear) a pipe buffer
 *
 * \param file FILE* the handle of the file
 * \return void
 *
 */
void EBmonitor_flush(FILE* file)
{
    if(file == stdin)
    {
        _eb_monitor_stdin.tail = _eb_monitor_stdin.head;
        return;
    }

    if(file == stdout)
    {
        _eb_monitor_stdout.head = _eb_monitor_stdout.tail ;
    }
}


/** \brief Check if there is any data in the stdin pipe
 *
 * \return int boolean is data is available, 0 is no data
 *
 */
int EBmonitor_kbhit()
{
    return (_eb_monitor_stdin.tail != _eb_monitor_stdin.head);
}


/** \brief Write data to the stdout pipe
 *
 * \param file int the handle of the pipe
 * \param ptr char* to buffer with data
 * \param len int of the data in the buffer
 * \return int with the number of characters written
 *
 */
int _write(int file, char *ptr, int len)
{
    int charWritten = 0;

#ifndef NO_EBMON_INIT
    if(!_eb_initialized)
    {
        _eb_initialized = -1;
        ebmonitor_init();
    }
#endif

    int head = _eb_monitor_stdout.head;
    for (; len > 0; --len)
    {
        if(++head >= _eb_monitor_stdout.length)
           head = 0;

        if( head == _eb_monitor_stdout.tail)
            return charWritten;

        ( (char*)_eb_monitor_stdout.ptr)[head] = *ptr;
        _eb_monitor_stdout.head = head;

        ptr++;
        charWritten++;
    }

    return charWritten;
}


/** \brief Read data from the stdin pipe
 *
 * \param file int the handle of the pipe
 * \param ptr char* to buffer where the data must be written to
 * \param len int with the length of the buffer
 * \return int with the actual number of characters written into the buffer
 *
 */
int _read(int file, char *ptr, int len)
{
    int charRead = 0;

#ifndef NO_EBMON_INIT
    if(!_eb_initialized)
    {
        _eb_initialized = -1;
        ebmonitor_init();
    }
#endif

    if( _eb_monitor_stdin.tail == _eb_monitor_stdin.head)
        return -1;

    for (; len > 0; --len)
    {
        if(++_eb_monitor_stdin.tail >= _eb_monitor_stdin.length)
            _eb_monitor_stdin.tail = 0;

        *ptr = ((char*)_eb_monitor_stdin.ptr)[_eb_monitor_stdin.tail];
        ptr++;
        charRead++;

        if( _eb_monitor_stdin.tail == _eb_monitor_stdin.head)
            return charRead;
    }

    return charRead;
}

/*@}*/ /* end of group EBmonitor */



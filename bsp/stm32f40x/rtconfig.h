#ifndef RT_CONFIG_H__
#define RT_CONFIG_H__

/* Automatically generated file; DO NOT EDIT. */
/* RT-Thread Project Configuration */

/* RT-Thread Kernel */

#define RT_NAME_MAX 8
#define RT_ALIGN_SIZE 4
#define RT_THREAD_PRIORITY_32
#define RT_THREAD_PRIORITY_MAX 32
#define RT_TICK_PER_SECOND 100
#define RT_USING_OVERFLOW_CHECK
#define RT_USING_HOOK
#define RT_USING_IDLE_HOOK
#define RT_IDEL_HOOK_LIST_SIZE 4
#define IDLE_THREAD_STACK_SIZE 1024
#define RT_USING_TIMER_SOFT
#define RT_TIMER_THREAD_PRIO 4
#define RT_TIMER_THREAD_STACK_SIZE 512
#define RT_DEBUG
#define RT_DEBUG_INIT_CONFIG
#define RT_DEBUG_INIT 1
#define RT_DEBUG_MEM_CONFIG
#define RT_DEBUG_MEM 1
#define RT_DEBUG_SLAB_CONFIG
#define RT_DEBUG_SLAB 1
#define RT_DEBUG_MEMHEAP_CONFIG
#define RT_DEBUG_MEMHEAP 1
#define RT_DEBUG_MODULE_CONFIG
#define RT_DEBUG_MODULE 1

/* Inter-Thread communication */

#define RT_USING_SEMAPHORE
#define RT_USING_MUTEX
#define RT_USING_EVENT
#define RT_USING_MAILBOX
#define RT_USING_MESSAGEQUEUE

/* Memory Management */

#define RT_USING_MEMPOOL
#define RT_USING_MEMHEAP
#define RT_USING_SMALL_MEM
#define RT_USING_MEMTRACE
#define RT_USING_HEAP

/* Kernel Device Object */

#define RT_USING_DEVICE
#define RT_USING_INTERRUPT_INFO
#define RT_USING_CONSOLE
#define RT_CONSOLEBUF_SIZE 128
#define RT_CONSOLE_DEVICE_NAME "uart1"
#define RT_VER_NUM 0x40000
#define ARCH_ARM
#define ARCH_ARM_CORTEX_M
#define ARCH_ARM_CORTEX_M4

/* RT-Thread Components */

#define RT_USING_COMPONENTS_INIT
#define RT_USING_USER_MAIN
#define RT_MAIN_THREAD_STACK_SIZE 2048
#define RT_MAIN_THREAD_PRIORITY 10

/* C++ features */


/* Command shell */

#define RT_USING_FINSH
#define FINSH_THREAD_NAME "tshell"
#define FINSH_USING_HISTORY
#define FINSH_HISTORY_LINES 5
#define FINSH_USING_SYMTAB
#define FINSH_USING_DESCRIPTION
#define FINSH_THREAD_PRIORITY 20
#define FINSH_THREAD_STACK_SIZE 4096
#define FINSH_CMD_SIZE 80
#define FINSH_USING_MSH
#define FINSH_USING_MSH_DEFAULT
#define FINSH_ARG_MAX 10

/* Device virtual file system */

#define RT_USING_DFS
#define DFS_FILESYSTEMS_MAX 2
#define DFS_FILESYSTEM_TYPES_MAX 2
#define DFS_FD_MAX 16
#define RT_USING_DFS_ELMFAT

/* elm-chan's FatFs, Generic FAT Filesystem Module */

#define RT_DFS_ELM_CODE_PAGE 936
#define RT_DFS_ELM_WORD_ACCESS
#define RT_DFS_ELM_USE_LFN_3
#define RT_DFS_ELM_USE_LFN 3
#define RT_DFS_ELM_MAX_LFN 255
#define RT_DFS_ELM_DRIVES 2
#define RT_DFS_ELM_MAX_SECTOR_SIZE 512
#define RT_DFS_ELM_REENTRANT
#define RT_USING_DFS_DEVFS

/* Device Drivers */

#define RT_USING_DEVICE_IPC
#define RT_PIPE_BUFSZ 512
#define RT_USING_SERIAL
#define RT_SERIAL_USING_DMA
#define RT_USING_PIN
#define RT_USING_SPI
#define RT_USING_W25QXX

/* Using WiFi */


/* Using USB */


/* POSIX layer and C standard library */


/* Network */

/* Socket abstraction layer */


/* light weight TCP/IP stack */


/* Modbus master and slave stack */


/* AT commands */


/* VBUS(Virtual Software BUS) */


/* Utilities */


/* ARM CMSIS */


/* RT-Thread online packages */

/* IoT - internet of things */


/* Wi-Fi */

/* Marvell WiFi */


/* Wiced WiFi */


/* IoT Cloud */


/* security packages */


/* language packages */


/* multimedia packages */


/* tools packages */

#define PKG_USING_SYSTEMVIEW
#define PKG_SYSVIEW_APP_NAME "RT-Thread Trace"
#define PKG_SYSVIEW_DEVICE_NAME "Cortex-M4"
#define PKG_SYSVIEW_TIMESTAMP_FREQ 0
#define PKG_SYSVIEW_CPU_FREQ 0
#define PKG_SYSVIEW_RAM_BASE 0x20000000
#define PKG_SYSVIEW_EVENTID_OFFSET 32
#define PKG_SYSVIEW_USE_CYCCNT_TIMESTAMP
#define PKG_SYSVIEW_SYSDESC0 "I#15=SysTick"
#define PKG_SYSVIEW_SYSDESC1 ""
#define PKG_SYSVIEW_SYSDESC2 ""

/* Segger RTT configuration */

#define PKG_SEGGER_RTT_MAX_NUM_UP_BUFFERS 3
#define PKG_SEGGER_RTT_MAX_NUM_DOWN_BUFFERS 3
#define PKG_SEGGER_RTT_BUFFER_SIZE_UP 1024
#define PKG_SEGGER_RTT_BUFFER_SIZE_DOWN 16
#define PKG_SEGGER_RTT_PRINTF_BUFFER_SIZE 64
#define PKG_SEGGER_RTT_AS_SERIAL_DEVICE
#define PKG_SERIAL_DEVICE_NAME "segger"
#define PKG_SEGGER_RTT_MODE_ENABLE_NO_BLOCK_SKIP
#define PKG_SEGGER_RTT_MAX_INTERRUPT_PRIORITY 0x20

/* SystemView buffer configuration */

#define PKG_SEGGER_SYSVIEW_RTT_BUFFER_SIZE 1024
#define PKG_SEGGER_SYSVIEW_RTT_CHANNEL 1
#define PKG_SEGGER_SYSVIEW_USE_STATIC_BUFFER

/* SystemView Id configuration */

#define PKG_SEGGER_SYSVIEW_ID_BASE 0x10000000
#define PKG_SEGGER_SYSVIEW_ID_SHIFT 2
#define PKG_USING_SYSTEMVIEW_LATEST_VERSION

/* system packages */


/* peripheral libraries and drivers */


/* miscellaneous packages */


/* samples: kernel and components samples */

#define SOC_STM32F4
#define RT_USING_UART1

#endif

/*
 *  serial.c UART driver
 *
 * COPYRIGHT (C) 2014, Shanghai Real-Thread Technology Co., Ltd
 *
 *  This file is part of RT-Thread (http://www.rt-thread.org)
 *
 *  All rights reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2014-04-07     Grissiom     first version
 */
/* 4MB in size */
/* VMM Memory Map:
 *
 * --- VMM_BEGIN --- +------+
 *     .vectors      | 4KB  |
 *   .text.share     |      |
 * ----------------- +      |
 * guest vector page | 4KB  |
 * ----------------- +      |
 *   .data.share     | 4KB  |
 * ----------------- +      |
 *    .bss.share     | 4KB  |
 * -- SHARE_BASE  -- +      | 1MB
 *  shared context          | shared region
 * -----------------        |
 *      blabla...           |
 * ----------------- +------+
 *  vmm text                |
 *      rodata              |
 *      blabla...           |
 * -----------------        | private region
 *  vmm data                |
 * -----------------        |
 *  vmm bss                 |
 * ---- VMM_END ---- +------+
 *
 */
/* 1MB is one level one page table entry, if we want to page table to be
 * simple(avoid TLB miss), we could allocate 1MB for shared memory. */
/* the size and position of shared code text */
/* the size and position of vector's page size in Linux */
/* the size and position of shared code data */
/* the size and position of shared code bss */
/* the size and position of shared code bss */
/* the size of FIQ stack page size in RT-Thread */
/* the size of IRQ stack page size in RT-Thread */
/* the max number of iomap entries */
/*#define RT_VMM_USING_DOMAIN*/
OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)
SECTIONS
{
    . = (0xE0000000 - 0x400000);
    __text_start = .;
    .text.share :
    {
        KEEP(*(.vectors))
        *(.text.isr)
        *(.vmm_glue)
    }
    ASSERT(SIZEOF(.text.share) <= 4096, ".text.share too big")
    . = (0xE0000000 - 0x400000) + 4096;
    /* the vectore page is saved here
     * {
     * }
     */
    . = (((0xE0000000 - 0x400000) + 4096) + 4096);
    .data.share :
    {
        __data_share_start = .;
        *(.data.share*)
        __data_share_end = .;
    }
    ASSERT(SIZEOF(.data.share) <= (4096), ".data.share is too big")
    . = ((((0xE0000000 - 0x400000) + 4096) + 4096) + 4096);
    .bss.share :
    {
        __bss_share_start = .;
        *(.bss.share*)
        __bss_share_end = .;
    }
    ASSERT(SIZEOF(.bss.share) <= (4096), ".bss.share is too big")
    . = (((((0xE0000000 - 0x400000) + 4096) + 4096) + 4096) + 4096);
    .vmm.share :
    {
        /* the vmm context goes here */
        __vmm_share_start = .;
        *(.vmm.share*)
        __vmm_share_end = .;
    }
    ASSERT(SIZEOF(.vmm.share) <= (((1024*1024) - 4096 - 4096 - 4096 - 4096)), "vmm share context is too big")
    . = (0xE0000000 - 0x400000) + (1024*1024);
    .text :
    {
        *(.vmm_init)
        *(.text)
        *(.text.*)
        /* section information for finsh shell */
        . = ALIGN(4);
        __fsymtab_start = .;
        KEEP(*(FSymTab))
        __fsymtab_end = .;
        . = ALIGN(4);
        __vsymtab_start = .;
        KEEP(*(VSymTab))
        __vsymtab_end = .;
        . = ALIGN(4);
        /* section information for modules */
        . = ALIGN(4);
        __rtmsymtab_start = .;
        KEEP(*(RTMSymTab))
        __rtmsymtab_end = .;
        /* section information for initialization */
        . = ALIGN(4);
        __rt_init_start = .;
        KEEP(*(SORT(.rti_fn*)))
        __rt_init_end = .;
    }
    __text_end = .;
    __rodata_start = .;
    .rodata : { *(.rodata) *(.rodata.*) }
    __rodata_end = .;
    . = ALIGN(4);
    .ctors :
    {
        PROVIDE(__ctors_start__ = .);
        KEEP(*(SORT(.ctors.*)))
        KEEP(*(.ctors))
        PROVIDE(__ctors_end__ = .);
    }
    .dtors :
    {
        PROVIDE(__dtors_start__ = .);
        KEEP(*(SORT(.dtors.*)))
        KEEP(*(.dtors))
        PROVIDE(__dtors_end__ = .);
    }
    __data_start = .;
    . = ALIGN(8);
    .data :
    {
        *(.data)
        *(.data.*)
    }
    __data_end = .;
    . = ALIGN(8);
    __bss_start = __data_end;
    .bss :
    {
    vmm_stack_start = .;
    . = vmm_stack_start + 1024;
    vmm_stack_end = .;
    *(.bss)
    *(.bss.*)
    *(COMMON)
    . = ALIGN(4);
    }
    . = ALIGN(4);
    __bss_end = .;
    /* Stabs debugging sections.  */
    .stab 0 : { *(.stab) }
    .stabstr 0 : { *(.stabstr) }
    .stab.excl 0 : { *(.stab.excl) }
    .stab.exclstr 0 : { *(.stab.exclstr) }
    .stab.index 0 : { *(.stab.index) }
    .stab.indexstr 0 : { *(.stab.indexstr) }
    .comment 0 : { *(.comment) }
    _end = .;
}

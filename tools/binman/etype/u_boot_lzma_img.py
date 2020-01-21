# SPDX-License-Identifier: GPL-2.0
# Copyright (C) 2020 MediaTek Inc. All Rights Reserved.
# Author: Weijie Gao <weijie.gao@mediatek.com>
#
# Entry-type module for U-Boot legacy image with contents compressed by LZMA
#

from entry import Entry
from blob import Entry_blob

class Entry_u_boot_lzma_img(Entry_blob):
    """U-Boot legacy image with contents compressed by LZMA

    Properties / Entry arguments:
        - filename: Filename of u-boot-lzma.img (default 'u-boot-lzma.img')

    This is the U-Boot binary as a packaged image, in legacy format. It has a
    header which allows it to be loaded at the correct address for execution.
    Its contents are compressed by LZMA.

    You should use FIT (Flat Image Tree) instead of the legacy image for new
    applications.
    """
    def __init__(self, section, etype, node):
        Entry_blob.__init__(self, section, etype, node)

    def GetDefaultFilename(self):
        return 'u-boot-lzma.img'

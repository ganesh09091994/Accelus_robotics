/*
 * Copyright (c) 2010, NXP Semiconductors N.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the NXP Semiconductors N.V. nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND  CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED  WARRANTIES,  INCLUDING,  BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF  MERCHANTABILITY  AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL NXP Semiconductors N.V. BE LIABLE FOR ANY
 * DIRECT, INDIRECT,  INCIDENTAL,  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY  THEORY  OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR  OTHERWISE)  ARISING  IN  ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * iap_driver.h
 * Demonstrate basic capability of In-Application Programming feature
 *
 * by: Daniel Widyanto <daniel.widyanto@nxp.com>
 */

#ifndef IAP_DRIVER_H_
#define IAP_DRIVER_H_

// #pragma once
void *iap_get_ptr(unsigned int sector, unsigned int offset);
int iap_read(unsigned int sector,unsigned int offset, void *buf,unsigned int size);
int iap_write(unsigned int sector,unsigned int offset, const void *buf,unsigned int size);
int iap_cmp(unsigned int sector, unsigned int offset, const void *buf, unsigned int size);
int iap_copy(unsigned int dest_sector, unsigned int src_sector, 
        unsigned int offset, unsigned int size);
int iap_erase(unsigned int sector);
int iap_blank_check(unsigned int sector);
int iap_read_serial(void);
int iap_read_version(void);
int iap_read_id(void);
void iap_init(void);


#endif /* IAP_DRIVER_H_ */
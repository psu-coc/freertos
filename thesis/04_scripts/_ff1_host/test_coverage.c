/* Host test: run the unmodified ff1_speck.c/speck.c from branch `ff1`
 * exactly as SECURE_RTSMARM_FF1_Speck_ShuffledHMAC_secure does, and count
 * how many distinct blocks are visited for each block size. */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "ff1_speck.h"

#define TOTAL_SIZE 0x80000u

static uint8_t seen[TOTAL_SIZE / 64];

int main(void)
{
    const uint32_t sizes[] = {64, 128, 256, 512, 1024, 2048, 4096};
    uint8_t key16[16];
    for (int i = 0; i < 16; i++) key16[i] = (uint8_t)(0xA5 ^ (i * 17));
    const uint8_t challenge[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    uint32_t tweak = ((uint32_t)challenge[0] << 24) | ((uint32_t)challenge[1] << 16)
                   | ((uint32_t)challenge[2] << 8) | (uint32_t)challenge[3];

    static FF1_Key_Speck k;
    FF1_SetKey_Speck(&k, key16);

    for (size_t s = 0; s < sizeof(sizes) / sizeof(sizes[0]); s++) {
        uint32_t blocks = TOTAL_SIZE / sizes[s];
        uint32_t hashed = 0, distinct = 0;
        memset(seen, 0, sizeof(seen));
        for (uint32_t i = 0; i < blocks; i++) {
            uint32_t idx = FF1Permute_Speck(i, blocks, &k, tweak);
            if (idx >= blocks) continue;
            hashed++;
            if (!seen[idx]) { seen[idx] = 1; distinct++; }
        }
        printf("B=%4u n=%4u  hash calls=%4u  distinct blocks measured=%4u (%.1f%%)  first never-measured:",
               sizes[s], blocks, hashed, distinct, 100.0 * distinct / blocks);
        int shown = 0;
        for (uint32_t b = 0; b < blocks && shown < 6; b++)
            if (!seen[b]) { printf(" %u", b); shown++; }
        if (!shown) printf(" none");
        printf("\n");
    }
    return 0;
}

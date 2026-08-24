//! Port of box3d-cpp-reference/src/rapidhash.h
//!
//! ```text
//! rapidhash V3 - Very fast, high quality, platform-independent hashing algorithm.
//!
//! Based on 'wyhash', by Wang Yi <godspeed_china@yeah.net>
//!
//! Copyright (C) 2025 Nicolas De Carli
//!
//! Permission is hereby granted, free of charge, to any person obtaining a copy
//! of this software and associated documentation files (the "Software"), to deal
//! in the Software without restriction, including without limitation the rights
//! to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//! copies of the Software, and to permit persons to whom the Software is
//! furnished to do so, subject to the following conditions:
//!
//! The above copyright notice and this permission notice shall be included in all
//! copies or substantial portions of the Software.
//!
//! THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//! IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//! FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//! AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//! LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//! OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//! SOFTWARE.
//!
//! You can contact the author at:
//!   - rapidhash source repository: https://github.com/Nicoshev/rapidhash
//! ```
//!
//! Configuration ported: the defaults the Box3D build compiles with.
//! - `RAPIDHASH_COMPACT` (`RAPIDHASH_UNROLLED` is not defined), so the 112-byte
//!   compact main loop is used.
//! - `RAPIDHASH_FAST` (`RAPIDHASH_PROTECTED` is not defined), so `rapid_mum`
//!   overwrites rather than xors its outputs.
//! - `RAPIDHASH_LITTLE_ENDIAN`: the C reads native-endian words through `memcpy`
//!   on little-endian hosts, and byte-swaps them on big-endian hosts so that both
//!   see the little-endian interpretation. The port reads explicit little-endian
//!   words, so it is byte-identical to upstream on every target.
//!
//! Box3D only calls the full `rapidhash` entry point with the default seed and
//! secret, so `rapidhashMicro` / `rapidhashNano` are not ported.

/// Default secret parameters.
pub const RAPID_SECRET: [u64; 8] = [
    0x2d358dccaa6c78a5,
    0x8bb84b93962eacc9,
    0x4b33a62ed433d4a3,
    0x4d5a2da51de1aa47,
    0xa0761d6478bd642f,
    0xe7037ed1a0b428db,
    0x90ed1765281c388c,
    0xaaaaaaaaaaaaaaaa,
];

/// 64*64 -> 128bit multiply function.
///
/// Calculates 128-bit C = *A * *B.
///
/// When RAPIDHASH_FAST is defined (the ported configuration):
/// Overwrites A contents with C's low 64 bits.
/// Overwrites B contents with C's high 64 bits.
#[inline]
fn rapid_mum(a: &mut u64, b: &mut u64) {
    // C uses __uint128_t when available; u128 is the exact equivalent.
    let r = (*a as u128) * (*b as u128);
    *a = r as u64;
    *b = (r >> 64) as u64;
}

/// Multiply and xor mix function.
///
/// Calculates 128-bit C = A * B.
/// Returns 64-bit xor between high and low 64 bits of C.
#[inline]
fn rapid_mix(a: u64, b: u64) -> u64 {
    let mut a = a;
    let mut b = b;
    rapid_mum(&mut a, &mut b);
    a ^ b
}

/// Read functions (little-endian configuration).
#[inline]
fn rapid_read64(p: &[u8]) -> u64 {
    u64::from_le_bytes(p[0..8].try_into().unwrap())
}

#[inline]
fn rapid_read32(p: &[u8]) -> u64 {
    u32::from_le_bytes(p[0..4].try_into().unwrap()) as u64
}

/// rapidhash main function.
///
/// * `key` - Buffer to be hashed.
/// * `seed` - 64-bit seed used to alter the hash result predictably.
/// * `secret` - Triplet of 64-bit secrets used to alter hash result predictably.
///
/// Returns a 64-bit hash.
pub fn rapidhash_internal(key: &[u8], seed: u64, secret: &[u64; 8]) -> u64 {
    let p = key;
    let len = key.len();
    let mut seed = seed ^ rapid_mix(seed ^ secret[2], secret[1]);
    let mut a: u64 = 0;
    let mut b: u64 = 0;
    let mut i = len;

    // Offset of the C `p` cursor into `key`.
    let mut pos = 0usize;

    if len <= 16 {
        if len >= 4 {
            seed ^= len as u64;
            if len >= 8 {
                a = rapid_read64(p);
                b = rapid_read64(&p[len - 8..]);
            } else {
                a = rapid_read32(p);
                b = rapid_read32(&p[len - 4..]);
            }
        } else if len > 0 {
            a = ((p[0] as u64) << 45) | p[len - 1] as u64;
            b = p[len >> 1] as u64;
        }
        // else: len == 0, a and b stay zero (C: a = b = 0)
    } else {
        if len > 112 {
            let mut see1 = seed;
            let mut see2 = seed;
            let mut see3 = seed;
            let mut see4 = seed;
            let mut see5 = seed;
            let mut see6 = seed;

            // RAPIDHASH_COMPACT
            loop {
                seed = rapid_mix(
                    rapid_read64(&p[pos..]) ^ secret[0],
                    rapid_read64(&p[pos + 8..]) ^ seed,
                );
                see1 = rapid_mix(
                    rapid_read64(&p[pos + 16..]) ^ secret[1],
                    rapid_read64(&p[pos + 24..]) ^ see1,
                );
                see2 = rapid_mix(
                    rapid_read64(&p[pos + 32..]) ^ secret[2],
                    rapid_read64(&p[pos + 40..]) ^ see2,
                );
                see3 = rapid_mix(
                    rapid_read64(&p[pos + 48..]) ^ secret[3],
                    rapid_read64(&p[pos + 56..]) ^ see3,
                );
                see4 = rapid_mix(
                    rapid_read64(&p[pos + 64..]) ^ secret[4],
                    rapid_read64(&p[pos + 72..]) ^ see4,
                );
                see5 = rapid_mix(
                    rapid_read64(&p[pos + 80..]) ^ secret[5],
                    rapid_read64(&p[pos + 88..]) ^ see5,
                );
                see6 = rapid_mix(
                    rapid_read64(&p[pos + 96..]) ^ secret[6],
                    rapid_read64(&p[pos + 104..]) ^ see6,
                );
                pos += 112;
                i -= 112;

                if i <= 112 {
                    break;
                }
            }

            seed ^= see1;
            see2 ^= see3;
            see4 ^= see5;
            seed ^= see6;
            see2 ^= see4;
            seed ^= see2;
        }

        if i > 16 {
            seed = rapid_mix(
                rapid_read64(&p[pos..]) ^ secret[2],
                rapid_read64(&p[pos + 8..]) ^ seed,
            );
            if i > 32 {
                seed = rapid_mix(
                    rapid_read64(&p[pos + 16..]) ^ secret[2],
                    rapid_read64(&p[pos + 24..]) ^ seed,
                );
                if i > 48 {
                    seed = rapid_mix(
                        rapid_read64(&p[pos + 32..]) ^ secret[1],
                        rapid_read64(&p[pos + 40..]) ^ seed,
                    );
                    if i > 64 {
                        seed = rapid_mix(
                            rapid_read64(&p[pos + 48..]) ^ secret[1],
                            rapid_read64(&p[pos + 56..]) ^ seed,
                        );
                        if i > 80 {
                            seed = rapid_mix(
                                rapid_read64(&p[pos + 64..]) ^ secret[2],
                                rapid_read64(&p[pos + 72..]) ^ seed,
                            );
                            if i > 96 {
                                seed = rapid_mix(
                                    rapid_read64(&p[pos + 80..]) ^ secret[1],
                                    rapid_read64(&p[pos + 88..]) ^ seed,
                                );
                            }
                        }
                    }
                }
            }
        }

        a = rapid_read64(&p[pos + i - 16..]) ^ i as u64;
        b = rapid_read64(&p[pos + i - 8..]);
    }

    a ^= secret[1];
    b ^= seed;
    rapid_mum(&mut a, &mut b);
    rapid_mix(a ^ secret[7], b ^ secret[1] ^ i as u64)
}

/// rapidhash seeded hash function.
///
/// Calls [`rapidhash_internal`] using provided parameters and default secrets.
///
/// Returns a 64-bit hash.
pub fn rapidhash_with_seed(key: &[u8], seed: u64) -> u64 {
    rapidhash_internal(key, seed, &RAPID_SECRET)
}

/// rapidhash general purpose hash function.
///
/// Calls [`rapidhash_with_seed`] using provided parameters and the default seed.
///
/// Returns a 64-bit hash.
pub fn rapidhash(key: &[u8]) -> u64 {
    rapidhash_with_seed(key, 0)
}

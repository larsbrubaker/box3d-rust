//! Traditional Quicksort that mostly follows [Sedgewick 1978].
//!
//! Port of `box3d-cpp-reference/src/qsort.h` (Copyright (c) 2013, 2017 Alexey Tourbin, MIT
//! license). C expands the `QSORT( N, LESS, SWAP )` macro inline with user `LESS( i, j )` and
//! `SWAP( i, j )` primitives on array indices. The port is a generic function over a slice whose
//! `less` closure compares `a[i]` to `a[j]`, and whose swaps are slice swaps.
//!
//! The algorithm is ported step for step (median of three on the second, middle, and last items,
//! insertion sort below [`Q_THRESH`], larger subfile pushed to the stack), so the sequence of
//! comparisons and swaps matches C and so does the result for comparators with ties, where a
//! different sort could order equal keys differently.
//!
//! SPDX-FileCopyrightText: 2013, 2017 Alexey Tourbin
//! SPDX-License-Identifier: MIT

/// When the size of [q_l,q_r], i.e. q_r-q_l+1, is greater than or equal to Q_THRESH, the
/// algorithm performs recursive partitioning. When the size drops below Q_THRESH, the algorithm
/// switches to insertion sort. The minimum valid value is probably 5 (with 5 items, the second
/// and the middle items, the middle itself being rounded down, are distinct). (Q_THRESH)
const Q_THRESH: usize = 16;

/// Sort 3 elements. (Q_SORT3)
fn q_sort3<T>(a: &mut [T], a1: usize, a2: usize, a3: usize, less: &mut impl FnMut(&T, &T) -> bool) {
    if less(&a[a2], &a[a1]) {
        if less(&a[a3], &a[a2]) {
            a.swap(a1, a3);
        } else {
            a.swap(a1, a2);
            if less(&a[a3], &a[a2]) {
                a.swap(a2, a3);
            }
        }
    } else if less(&a[a3], &a[a2]) {
        a.swap(a2, a3);
        if less(&a[a2], &a[a1]) {
            a.swap(a1, a2);
        }
    }
}

/// Partition [q_l,q_r] around a pivot. After partitioning, [q_l,q_j] are the elements that are
/// less than or equal to the pivot, while [q_i,q_r] are the elements greater than or equal to
/// the pivot. Returns (q_i, q_j). (Q_PARTITION)
fn q_partition<T>(
    a: &mut [T],
    q_l: usize,
    q_r: usize,
    less: &mut impl FnMut(&T, &T) -> bool,
) -> (usize, usize) {
    // The middle element, not to be confused with the median.
    let q_m = q_l + ((q_r - q_l) >> 1);

    // Reorder the second, the middle, and the last items. As [Edelkamp Weiss 2016] explain,
    // using the second element instead of the first one helps avoid bad behaviour for
    // decreasingly sorted arrays. This method is used in recent versions of gcc's std::sort,
    // see gcc bug 58437#c13, although the details are somewhat different (cf. #c14).
    q_sort3(a, q_l + 1, q_m, q_r, less);

    // Place the median at the beginning.
    a.swap(q_l, q_m);

    // Partition [q_l+2, q_r-1] around the median which is in q_l. q_i and q_j are initially off
    // by one, they get decremented in the do-while loops.
    let mut q_i = q_l + 1;
    let mut q_j = q_r;
    loop {
        loop {
            q_i += 1;
            if !less(&a[q_i], &a[q_l]) {
                break;
            }
        }
        loop {
            q_j -= 1;
            if !less(&a[q_l], &a[q_j]) {
                break;
            }
        }

        // Sedgewick says "until j < i"
        if q_i >= q_j {
            break;
        }

        a.swap(q_i, q_j);
    }

    // Compensate for the i==j case.
    q_i = q_j + 1;

    // Put the median to its final place.
    a.swap(q_l, q_j);

    // The median is not part of the left subfile.
    q_j -= 1;

    (q_i, q_j)
}

/// Insertion sort is applied to small subfiles - this is contrary to Sedgewick's suggestion to
/// run a separate insertion sort pass after the partitioning is done. (Q_INSERTION_SORT)
fn q_insertion_sort<T>(a: &mut [T], q_l: usize, q_r: usize, less: &mut impl FnMut(&T, &T) -> bool) {
    // For each item starting with the second...
    let mut q_i = q_l + 1;
    while q_i <= q_r {
        // move it down the array so that the first part is sorted.
        let mut q_j = q_i;
        while q_j > q_l && less(&a[q_j], &a[q_j - 1]) {
            a.swap(q_j, q_j - 1);
            q_j -= 1;
        }
        q_i += 1;
    }
}

/// Sort `a[0..n]` with the `QSORT( N, LESS, SWAP )` algorithm, where `less( x, y )` is the
/// comparison C writes as `LESS( i, j )`. (QSORT)
pub(crate) fn qsort<T>(a: &mut [T], n: usize, mut less: impl FnMut(&T, &T) -> bool) {
    if n <= 1 {
        return;
    }

    debug_assert!(n <= a.len());

    // The main loop. (Q_LOOP)
    let mut q_l: usize = 0;
    let mut q_r: usize = n - 1;

    // The subfile stack. C sizes it 48 frames for a 64-bit index and count.
    let mut q_st: [(usize, usize); 48] = [(0, 0); 48];
    let mut q_sp: usize = 0;

    loop {
        if q_r - q_l + 1 >= Q_THRESH {
            let (q_i, q_j) = q_partition(a, q_l, q_r, &mut less);

            // Now have two subfiles: [q_l,q_j] and [q_i,q_r]. Dealing with them depends on
            // which one is bigger. (Q_SUBFILES assumes the first subfile is not smaller than
            // the second.)
            let (q_l1, q_r1, q_l2, q_r2) = if q_j - q_l >= q_r - q_i {
                (q_l, q_j, q_i, q_r)
            } else {
                (q_i, q_r, q_l, q_j)
            };

            if q_l2 == q_r2 {
                // If the second subfile is only a single element, it needs no further
                // processing. The first subfile will be processed on the next iteration (both
                // subfiles cannot be only a single element, due to Q_THRESH).
                q_l = q_l1;
                q_r = q_r1;
            } else {
                // Otherwise, both subfiles need processing. Push the larger subfile onto the
                // stack.
                q_st[q_sp] = (q_l1, q_r1);
                q_sp += 1;

                // Process the smaller subfile on the next iteration.
                q_l = q_l2;
                q_r = q_r2;
            }
        } else {
            q_insertion_sort(a, q_l, q_r, &mut less);

            // Pop subfiles from the stack, until it gets empty.
            if q_sp == 0 {
                break;
            }

            q_sp -= 1;
            q_l = q_st[q_sp].0;
            q_r = q_st[q_sp].1;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::qsort;

    /// Sorts match a reference sort on keys without ties, across the insertion-only and the
    /// partitioning sizes.
    #[test]
    fn qsort_sorts_distinct_keys() {
        let mut state = 12345u32;
        for n in [0usize, 1, 2, 3, 15, 16, 17, 40, 257] {
            let mut values: Vec<u32> = (0..n)
                .map(|_| {
                    state = state.wrapping_mul(1664525).wrapping_add(1013904223);
                    state
                })
                .collect();
            let mut expected = values.clone();
            expected.sort_unstable();
            qsort(&mut values, n, |a, b| a < b);
            assert_eq!(values, expected);
        }
    }

    /// Equal keys: the sorted key order is right whatever their arrangement.
    #[test]
    fn qsort_sorts_keys_with_ties() {
        let mut values: Vec<(u32, usize)> = (0..100).map(|i| ((i * 7 % 5) as u32, i)).collect();
        qsort(&mut values, 100, |a, b| a.0 < b.0);
        for i in 1..values.len() {
            assert!(values[i - 1].0 <= values[i].0);
        }
    }
}

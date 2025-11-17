use core::mem::{self, MaybeUninit};

pub struct OnDrop<F: FnOnce()> {
    f: MaybeUninit<F>,
}

impl<F: FnOnce()> OnDrop<F> {
    pub fn new(f: F) -> Self {
        Self {
            f: MaybeUninit::new(f),
        }
    }

    pub fn defuse(self) {
        mem::forget(self)
    }
}

impl<F: FnOnce()> Drop for OnDrop<F> {
    fn drop(&mut self) {
        unsafe { self.f.as_ptr().read()() }
    }
}

pub trait ArrayZipWith<Other, ResItem, Res> {
    type Item;
    type OtherItem;
    fn zip_with(self, other: Other, f: impl FnMut(Self::Item, Self::OtherItem) -> ResItem) -> Res;
}

impl<T, U, V, const N: usize> ArrayZipWith<[U; N], V, [V; N]> for [T; N] {
    type Item = T;
    type OtherItem = U;

    fn zip_with(self, other: [U; N], mut f: impl FnMut(T, U) -> V) -> [V; N] {
        let mut other = other.into_iter();
        self.map(move |x| {
            let y =
                // SAFETY: Types guarantee that the lengths match
                unsafe { other.next().unwrap_unchecked() };
            f(x, y)
        })
    }
}

#[allow(dead_code)] // maybe we'll use this someday
pub trait ArrayUnzip<T, U, const N: usize> {
    fn unzip(self) -> ([T; N], [U; N]);
}

impl<T, U, const N: usize> ArrayUnzip<T, U, N> for [(T, U); N] {
    fn unzip(self) -> ([T; N], [U; N]) {
        use core::mem::{self, MaybeUninit};

        // SAFETY: In both cases, MaybeUninit<[MaybeUninit<_>; N]> has the same layout as
        // [MaybeUninit<_>; N]
        let mut first: [MaybeUninit<T>; N] = unsafe { MaybeUninit::uninit().assume_init() };
        let mut snd: [MaybeUninit<U>; N] = unsafe { MaybeUninit::uninit().assume_init() };

        for (idx, (t, u)) in self.into_iter().enumerate() {
            first[idx] = MaybeUninit::new(t);
            snd[idx] = MaybeUninit::new(u);
        }

        // SAFETY: By this point, we've initialized each element of both array (because the lengths
        // are guaranteed to match)
        unsafe { (mem::transmute_copy(&first), mem::transmute_copy(&snd)) }
    }
}

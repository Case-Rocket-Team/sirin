use core::{cell::UnsafeCell, future::{poll_fn, Future}, marker::PhantomData, mem::{self, ManuallyDrop, MaybeUninit}, ops::{Deref, DerefMut}, pin::Pin, ptr::null_mut, sync::atomic::{AtomicBool, AtomicPtr, Ordering}, task::Poll};

use embassy_sync::waitqueue::AtomicWaker;

use crate::flash::Flash;

pub struct InnerCoroutine<T> {
    resource: *mut T,
    should_yield: AtomicBool,
    should_yield_waker: AtomicWaker,
    has_yielded: AtomicBool,
    has_yielded_waker: AtomicWaker
}

impl <T> InnerCoroutine<T> {
    pub fn new(resource: *mut T) -> Self {
        InnerCoroutine {
            resource: resource,
            should_yield: AtomicBool::new(false),
            should_yield_waker: AtomicWaker::new(),
            has_yielded: AtomicBool::new(true),
            has_yielded_waker: AtomicWaker::new()
        }
    }

    pub fn yield_now(&self) -> impl Future<Output = ()> + '_ {
        poll_fn(move |cx| {
            if self.should_yield.load(Ordering::Relaxed) {
                self.should_yield_waker.register(cx.waker());
                self.has_yielded.store(true, Ordering::Relaxed);
                self.has_yielded_waker.wake();
                Poll::Pending
            } else {
                Poll::Ready(())
            }
        })
    }
}

pub struct CoroutineHandle<T: 'static> {
    pub(crate) coroutine: &'static InnerCoroutine<T>
}

impl <T> CoroutineHandle<T> {
    pub fn new(coroutine: &'static InnerCoroutine<T>) -> Self {
        Self {
            coroutine
        }
    }

    // It is UB if _resource is not THE SAME mutable reference as from before
    unsafe fn run<'a>(&'static mut self, _resource: &'a mut T) -> RunningCoroutine<'a, T> {
        self.coroutine.should_yield.store(false, Ordering::SeqCst);

        RunningCoroutine {
            coroutine: ManuallyDrop::new(&self.coroutine),
            _ref: PhantomData
        }
    }
}

pub struct RunningCoroutine<'a, T: 'static> {
    coroutine: ManuallyDrop<&'static InnerCoroutine<T>>,
    _ref: PhantomData<&'a mut T>
}

impl <'a, T> RunningCoroutine<'a, T> {
    fn stop(mut self) -> impl Future<Output = &'static mut T> {
        let coroutine = unsafe {
            ManuallyDrop::take(&mut self.coroutine)
        };

        mem::forget(self);
        coroutine.should_yield.store(true, Ordering::SeqCst);
        coroutine.should_yield_waker.wake();

        poll_fn(|cx| {
            if let Ok(true) = coroutine.has_yielded.compare_exchange(true, false, Ordering::SeqCst, Ordering::SeqCst) {
                unsafe {
                    Poll::Ready(&mut *coroutine.resource)
                }
            } else {
                coroutine.has_yielded_waker.register(cx.waker());
                Poll::Pending
            }
        })
    }
}

impl <'a, T: 'static> Drop for RunningCoroutine<'a, T> {
    fn drop(&mut self) {
        // prevents this type from being dropped
        const {
            panic!("This type cannot be dropped");
        }
    }
}

pub struct InsideCoroutine<T: 'static> {
    coroutine: &'static InnerCoroutine<T>
}

impl <T> InsideCoroutine<T> {
    pub async fn yield_now(&mut self) {
        self.coroutine.yield_now().await
    }
}

async fn test(flash: &'static mut Flash) {
    let _co = InnerCoroutine::<Flash>::new(flash as *mut Flash);
    let co = transmute_static_ref(&_co);
    let mut _handle = CoroutineHandle::new(&co);
    let handle = transmute_static_mut(&mut _handle);

    let running = unsafe {
        handle.run(flash)
    };

    running.stop().await;

    flash.debug();
}

fn transmute_static_ref<T>(refr: &T) -> &'static T {
    unsafe {
        mem::transmute(refr)
    }
}

fn transmute_static_mut<T>(refr: &mut T) -> &'static mut T {
    unsafe {
        mem::transmute(refr)
    }
}
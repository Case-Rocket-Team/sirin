use core::{future::Future, pin::Pin, task::{Context, Poll}};

pub fn yield_now() -> impl Future<Output = ()> {
    YieldNowFuture { yielded: false }
}

#[must_use = "futures do nothing unless you `.await` or poll them"]
struct YieldNowFuture {
    yielded: bool,
}

impl Future for YieldNowFuture {
    type Output = ();
    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        if self.yielded {
            Poll::Ready(())
        } else {
            self.yielded = true;
            cx.waker().wake_by_ref();
            Poll::Pending
        }
    }
}
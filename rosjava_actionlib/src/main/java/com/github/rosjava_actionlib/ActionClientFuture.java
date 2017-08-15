package com.github.rosjava_actionlib;

import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

/**
 * Created by lruegeme on 3/17/17.
 */
public class ActionClientFuture implements Future<Boolean> {

    private ClientStateMachine sm;

    ActionClientFuture(ClientStateMachine sm) {
        this.sm = sm;
    }

    @Override
    public boolean cancel(boolean b) {
        //logger.warn("cancel called. This does nothing, use action client to cancel goal");
        return false;
    }

    @Override
    public boolean isCancelled() {
        return isDone() && sm.getState() != ClientStateMachine.ClientStates.DONE;
    }

    @Override
    public boolean isDone() {
        return !sm.isRunning();
    }

    @Override
    public Boolean get() throws InterruptedException, ExecutionException {
        while (sm.isRunning()) {
            Thread.sleep(100);
        }
        return sm.getState() == ClientStateMachine.ClientStates.DONE;
    }

    @Override
    public Boolean get(long l, TimeUnit timeUnit) throws InterruptedException, ExecutionException, TimeoutException {
        long timeout = System.currentTimeMillis() + timeUnit.toMillis(l);
        while (sm.isRunning()) {
            if (System.currentTimeMillis() > timeout) throw new TimeoutException();
            else Thread.sleep(100);
        }
        return sm.getState() == ClientStateMachine.ClientStates.DONE;
    }
}


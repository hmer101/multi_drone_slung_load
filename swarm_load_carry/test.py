import asyncio, threading

class TestClass:
    def __init__(self):
        self.event_loop_a = asyncio.new_event_loop()
        self.event_loop_b = asyncio.new_event_loop()

        self.t1 = threading.Thread(target=lambda: self.run_loop(self.event_loop_a)).start() # These lines start the event loops and run them in other threads until those threads are shut down
        self.t2 = threading.Thread(target=lambda: self.run_loop(self.event_loop_b)).start()

    def hello(self, thread_name):
        print('hello from thread {}!'.format(thread_name))

    async def async_hello(self, thread_name='default'):
        print('hello from thread {}!'.format(thread_name))

        if thread_name == 'b':
            await asyncio.sleep(2)

        return thread_name


    def run_loop(self, loop):
        asyncio.set_event_loop(loop)
        loop.run_forever()
        print('Loop exited')

async def main():
    testClass = TestClass()

    # event_loop_a.call_soon_threadsafe(lambda: hello('a'))
    # event_loop_b.call_soon_threadsafe(lambda: hello('b'))

    #coro_a = async_hello()
    fut_a = asyncio.run_coroutine_threadsafe(testClass.async_hello('a'), testClass.event_loop_a)
    fut_b = asyncio.run_coroutine_threadsafe(testClass.async_hello('b'), testClass.event_loop_a)

    print('after other threads init')


    print('waiting for futures done')
    #assert fut_a.result(None) == 'a'
    result_a = fut_a.result(None)
    print(f'RESULT A: {result_a}')

    result_b = fut_b.result(None)
    print(f'RESULT B: {result_b}')

    print('done - about to c')

    fut_c = asyncio.run_coroutine_threadsafe(testClass.async_hello('c'), testClass.event_loop_a)

    result_c = fut_c.result(None)
    print(f'RESULT C: {result_c}')

    #await fut_a
    #await fut_b

    testClass.event_loop_a.call_soon_threadsafe(testClass.event_loop_a.stop)
    testClass.event_loop_b.call_soon_threadsafe(testClass.event_loop_b.stop)

if __name__ =='__main__':
    asyncio.run(main())
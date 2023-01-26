import asyncio, rclpy

from rclpy.node import Node
from std_msgs.msg import Bool
from concurrent.futures import ThreadPoolExecutor

import rclpy.qos as qos
from rclpy.qos import QoSProfile


class Foo(Node):
    def __init__(self):
        super().__init__('foo')

        # Setup test subscriber
        qos_profile = QoSProfile(
            reliability=qos.ReliabilityPolicy.BEST_EFFORT,
            durability=qos.DurabilityPolicy.TRANSIENT_LOCAL,
            history=qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.test_sub = self.create_subscription(
            Bool,
            '/foo_sub',
            self.clbk,
            qos_profile)
        

    # Test callback function
    async def clbk(self, msg):
        self.get_logger().info(f'Received: {msg.data}. About to await fxn')
        # await asyncio.sleep(3)

        # This appears to run sleep in a separate thread.
        # executor = ThreadPoolExecutor(max_workers=1)
        # asyncio.get_event_loop().run_in_executor(executor, asyncio.run, asyncio.sleep(3))
        # executor.shutdown(wait=True)

        # The above method doesn't work when getting a returned value
        # executor = ThreadPoolExecutor(max_workers=1)
        # asyncio.get_event_loop().run_in_executor(executor, asyncio.run, self.sleep_with_return())
        # executor.shutdown(wait=True)

        # Also doesn't wait for a generator
        executor = ThreadPoolExecutor(max_workers=1)
        asyncio.get_event_loop().run_in_executor(executor, asyncio.run, self.test_generator())
        executor.shutdown(wait=True)


        self.get_logger().info('Clbk complete')


    # Workaround failure cases
    async def sleep_with_return(self):
        await asyncio.sleep(3)

        return True
    
    async def test_generator(self):
        async for next_num in self.async_generator():
            print(f'next_num: {next_num}')
    
    async def async_generator(self):
        for i in range(10):
            # suspend and sleep a moment
            await asyncio.sleep(1)
            # yield a value to the caller
            yield i


async def main():
    rclpy.init()

    # Create node and spin
    foo = Foo()
    rclpy.spin(foo)

if __name__ == '__main__':
    asyncio.run(main())
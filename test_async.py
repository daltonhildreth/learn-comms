# This is a simplified version of what pso_opt is doing for its concurrent
# execution of comm_norender
import asyncio
import random

async def simulate(s, i=0):
    with open("%d.out" % s, "w") as out:
        with open("%d.err" % s, "w") as err:
            # you want create_subprocess_exec for pso_opt instead
            proc = await asyncio.create_subprocess_shell(
                "echo %ds" % (s + i), # can you specifiy this like Popen?
                stdout=out,
                stderr=err
            )
            await proc.wait()
            print("simulate exit with %s" % (proc.returncode), "after", 1 + i)
            return s + i

async def attempt(s):
    print ('ATTEMPTING MOTION')
    return await simulate(s, random.randint(2, 5))

async def PSO(s):
    print("in PSO", s)
    print("waiting for baseline")
    baseline = await simulate(s)
    a = [baseline]
    for i in range(5):
        print("PSO", s, "iter", i)
        for p in range(2):
            print("PSO", s, "iter", i, "waiting for p", p)
            p_r = await attempt(s)
            a += [p_r]
    return a

async def main():
    return await asyncio.gather(
        PSO(1),
        PSO(5),
        PSO(15)
    )

event_loop = asyncio.get_event_loop()
try:
    return_value = event_loop.run_until_complete(main())
    print("it returned", return_value)
finally:
    event_loop.close()

# every tutorial said to use this, I don't know why it doesn't exist
# asyncio.run(main())

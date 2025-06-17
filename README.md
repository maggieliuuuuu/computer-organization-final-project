# Computer Organization Final Project

### Q1
Follow the instructions in the slides.

### Q2
Modify the following files:
- Options.py
  ```python
  parser.add_option("--l3cache", action="store_true")
  ```
- Caches.py
  ```python
  class L3Cache(Cache):
    assoc = 8
    tag_latency = 20
    data_latency = 20
    response_latency = 20
    mshrs = 20
    tgts_per_mshr = 12
    write_buffers = 8
  ```
- Xbar.py
  ```python
  class L3XBar(CoherentXBar):
    width = 32

    frontend_latency = 1
    forward_latency = 0
    response_latency = 1
    snoop_response_latency = 1

    snoop_filter = SnoopFilter(lookup_latency = 0)

    point_of_unification = True
  ```
- BaseCPU.py
  ```python
  from XBar import L3XBar
  ```
  ```python
  def addThreeLevelCacheHierarchy(self, ic, dc, l3c, iwc=None, dwc=None,
                                  xbar=None):
    self.addPrivateSplitL1Caches(ic, dc, iwc, dwc)
    self.toL3Bus = L3XBar()
    self.connectCachedPorts(self.toL3Bus)
    self.l3cache = l3c
    self.toL3Bus.master = self.l3cache.cpu_side
    self._cached_ports = ['l3cache.mem_side']
  ```
- CacheConfig.py
  ```python
          dcache_class, icache_class, l2_cache_class, l3_cache_class, walk_cache_class = \
            O3_ARM_v7a_DCache, O3_ARM_v7a_ICache, O3_ARM_v7aL2, \
            O3_ARM_v7aL3, O3_ARM_v7aWalkCache
    else:
        dcache_class, icache_class, l2_cache_class, l3_cache_class, walk_cache_class = \
            L1_DCache, L1_ICache, L2Cache, L3Cache, None
  ```
  ```python
  if options.l2cache and options.l3cache:
        system.l2 = l2_cache_class(clk_domain=system.cpu_clk_domain,
                                   size=options.l2_size,
                                   assoc=options.l2_assoc)

	system.l3 = l3_cache_class(clk_domain=system.cpu_clk_domain,
                                   size=options.l3_size,
                                   assoc=options.l3_assoc)

        system.tol2bus = L2XBar(clk_domain = system.cpu_clk_domain)

	system.tol3bus = L3XBar(clk_domain = system.cpu_clk_domain)

        system.l2.cpu_side = system.tol2bus.master
        system.l2.mem_side = system.tol3bus.slave
	
	system.l3.cpu_side = system.tol3bus.master
        system.l3.mem_side = system.membus.slave

    elif options.l2cache:
        system.l2 = l2_cache_class(clk_domain=system.cpu_clk_domain,
                                   size=options.l2_size,
                                   assoc=options.l2_assoc)

        system.tol2bus = L2XBar(clk_domain = system.cpu_clk_domain)
        system.l2.cpu_side = system.tol2bus.master
        system.l2.mem_side = system.membus.slave
  ```

Command (add --l3cache):
```shell
./build/X86/gem5.opt configs/example/se.py -c tests/test-progs/hello/bin/x86/linux/hello --cpu-type=TimingSimpleCPU --caches --l2cache --l3cache --mem-type=NVMainMemory --nvmain-config=../NVmain/Config/PCM_ISSCC_2012_4GB.config > Q2_log.txt
```

### Q3
1. L3 cache size = 1MB (miss rate相同)
- 2-way:
```shell
./build/X86/gem5.opt configs/example/se.py -c tests/test-progs/benchmark/quicksort --cpu-type=TimingSimpleCPU --caches --l2cache --l3cache --l3_assoc=2 --l1i_size=32kB --l1d_size=32kB --l2_size=128kB --l3_size=1MB --mem-type=NVMainMemory --nvmain-config=../NVmain/Config/PCM_ISSCC_2012_4GB.config > Q3_2-way_1MB_log.txt 
```
- full-way:
```shell
./build/X86/gem5.opt configs/example/se.py -c tests/test-progs/benchmark/quicksort --cpu-type=TimingSimpleCPU --caches --l2cache --l3cache --l3_assoc=16384 --l1i_size=32kB --l1d_size=32kB --l2_size=128kB --l3_size=1MB --mem-type=NVMainMemory --nvmain-config=../NVmain/Config/PCM_ISSCC_2012_4GB.config > Q3_full-way_1MB_log.txt
```

2. L3 cache size = 512kB (miss rate有差異)
- 2-way:
```shell
./build/X86/gem5.opt configs/example/se.py -c tests/test-progs/benchmark/quicksort --cpu-type=TimingSimpleCPU --caches --l2cache --l3cache --l3_assoc=2 --l1i_size=32kB --l1d_size=32kB --l2_size=128kB --l3_size=512kB --mem-type=NVMainMemory --nvmain-config=../NVmain/Config/PCM_ISSCC_2012_4GB.config > Q3_2-way_512kB_log.txt
```
- full-way:
```shell
./build/X86/gem5.opt configs/example/se.py -c tests/test-progs/benchmark/quicksort --cpu-type=TimingSimpleCPU --caches --l2cache --l3cache --l3_assoc=8192 --l1i_size=32kB --l1d_size=32kB --l2_size=128kB --l3_size=512kB --mem-type=NVMainMemory --nvmain-config=../NVmain/Config/PCM_ISSCC_2012_4GB.config > Q3_full-way_512kB_log.txt
```

### Q4
Modify Caches.py (add the following line in L3Cache class):
```python
replacement_policy = Param.BaseReplacementPolicy(LFURP(),"Replacement policy")
```

測試:
program: quicksort
l3_assoc: 2
l3_size: 128kB, 256kB, 512kB, 1MB (其餘的cache size follow benchmark規定)
1MB時看不出差異

Command(take l3_size=128kB as an example):
```shell
./build/X86/gem5.opt configs/example/se.py -c tests/test-progs/benchmark/quicksort --cpu-type=TimingSimpleCPU --caches --l2cache --l3cache --l3_assoc=2 --l1i_size=32kB --l1d_size=32kB --l2_size=128kB --l3_size=128kB --mem-type=NVMainMemory --nvmain-config=../NVmain/Config/PCM_ISSCC_2012_4GB.config > Q4_lfu_128kB_log.txt
```

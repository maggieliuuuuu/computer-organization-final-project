# Computer Organization Final Project

### Q1
Follow the instructions in the slides.

Key points:
- gem5: an open source computer architecture simulator
- NVMain: an architectural level main memory simulator for emerging non-volatile memories
- gem5 and NVMain can be integrated to simulate non-volatile memory systems.

### Q2
Modify the following files:
- ./configs/common/Options.py
  ```python
  parser.add_option("--l3cache", action="store_true")
  ```
- ./configs/common/Caches.py
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
- ./src/mem/Xbar.py
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
- ./src/cpu/BaseCPU.py
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
- ./configs/common/CacheConfig.py
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

<!--Command (add --l3cache):
```shell
./build/X86/gem5.opt configs/example/se.py -c tests/test-progs/hello/bin/x86/linux/hello --cpu-type=TimingSimpleCPU --caches --l2cache --l3cache --mem-type=NVMainMemory --nvmain-config=../NVmain/Config/PCM_ISSCC_2012_4GB.config > NVMain.log
``` -->

Key points:
- BaseCPU.py: "The system can only use the L3 cache when the L2 cache is already used."
- The other files just imitate L2 cache to enable L3 cache.
- Add ```--l3cache``` in the command.

### Q3
```
program: quicksort
l3_assoc: 2-way, full-way(size/64)
l3_size: 128kB, 256kB, 512kB, 1MB (其餘的cache size follow benchmark規定)
```

<!--Command(take l3_size=1MB as an example):
- 2-way:
```shell
./build/X86/gem5.opt configs/example/se.py -c tests/test-progs/benchmark/quicksort --cpu-type=TimingSimpleCPU --caches --l2cache --l3cache --l3_assoc=2 --l1i_size=32kB --l1d_size=32kB --l2_size=128kB --l3_size=1MB --mem-type=NVMainMemory --nvmain-config=../NVmain/Config/PCM_ISSCC_2012_4GB.config > NVMain.log
```
- full-way:
```shell
./build/X86/gem5.opt configs/example/se.py -c tests/test-progs/benchmark/quicksort --cpu-type=TimingSimpleCPU --caches --l2cache --l3cache --l3_assoc=16384 --l1i_size=32kB --l1d_size=32kB --l2_size=128kB --l3_size=1MB --mem-type=NVMainMemory --nvmain-config=../NVmain/Config/PCM_ISSCC_2012_4GB.config > NVMain.log
``` -->

Key points:
- Associativity: add ```--l3_assoc=2``` (2-way) / ```--l3_assoc=size/64``` (full-way, since size of a cache block = 64 bytes) in the command
- Cache size: add ```--l1i_size=32kB --l1d_size=32kB --l2_size=128kB --l3_size=1MB``` in the command

Result & Analysis:
- 1MB時看不出2-way和full-way的miss rate差異 => size太大時associativity對miss rate的影響不大
- 512kB時full-way的miss rate較低 => full-way無conflict misses，因此可降低miss rate
- 128kB及256kB時2-way的miss rate較低 => size太小因此capacity misses影響較大

### Q4
Add fb_rp.hh (modified based on lfu_rp.hh and lru_rp.hh):
```c++
/**
 * Copyright (c) 2018 Inria
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Daniel Carvalho
 */

/**
 * @file
 * Declaration of a Least Frequently Used replacement policy.
 * The victim is chosen using the reference frequency. The least referenced
 * entry is always chosen to be evicted, regardless of the amount of times
 * it has been touched, or how long has passed since its last touch.
 */

#ifndef __MEM_CACHE_REPLACEMENT_POLICIES_FB_RP_HH__
#define __MEM_CACHE_REPLACEMENT_POLICIES_FB_RP_HH__

#include "mem/cache/replacement_policies/base.hh"

struct FBRPParams;

class FBRP : public BaseReplacementPolicy
{
  protected:
    /** FB-specific implementation of replacement data. */
    struct FBReplData : ReplacementData
    {
        /** Number of references to this entry since it was reset. */
        unsigned refCount;
	
	/** Tick on which the entry was last touched. */
        Tick lastTouchTick;

        /**
         * Default constructor. Invalidate data.
         */
        FBReplData() : refCount(0), lastTouchTick(0) {}
    };

  public:
    /** Convenience typedef. */
    typedef FBRPParams Params;

    /**
     * Construct and initiliaze this replacement policy.
     */
    FBRP(const Params *p);

    /**
     * Destructor.
     */
    ~FBRP() {}

    /**
     * Invalidate replacement data to set it as the next probable victim.
     * Clear the number of references.
     *
     * @param replacement_data Replacement data to be invalidated.
     */
    void invalidate(const std::shared_ptr<ReplacementData>& replacement_data)
                                                              const override;

    /**
     * Touch an entry to update its replacement data.
     * Increase number of references.
     *
     * @param replacement_data Replacement data to be touched.
     */
    void touch(const std::shared_ptr<ReplacementData>& replacement_data) const
                                                                     override;

    /**
     * Reset replacement data. Used when an entry is inserted.
     * Reset number of references.
     *
     * @param replacement_data Replacement data to be reset.
     */
    void reset(const std::shared_ptr<ReplacementData>& replacement_data) const
                                                                     override;

    /**
     * Find replacement victim using reference frequency.
     *
     * @param cands Replacement candidates, selected by indexing policy.
     * @return Replacement entry to be replaced.
     */
    ReplaceableEntry* getVictim(const ReplacementCandidates& candidates) const
                                                                     override;

    /**
     * Instantiate a replacement data entry.
     *
     * @return A shared pointer to the new replacement data.
     */
    std::shared_ptr<ReplacementData> instantiateEntry() override;
};

#endif // __MEM_CACHE_REPLACEMENT_POLICIES_FB_RP_HH__
```

Add fb_rp.cc (modified based on lfu_rp.cc and lru_rp.cc):
```c++
/**
 * Copyright (c) 2018 Inria
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Daniel Carvalho
 */

#include "mem/cache/replacement_policies/fb_rp.hh"

#include <cassert>
#include <memory>

#include "params/FBRP.hh"

FBRP::FBRP(const Params *p)
    : BaseReplacementPolicy(p)
{
}

void
FBRP::invalidate(const std::shared_ptr<ReplacementData>& replacement_data)
const
{
    // Reset reference count
    std::static_pointer_cast<FBReplData>(replacement_data)->refCount = 0;

    // Reset last touch timestamp
    std::static_pointer_cast<FBReplData>(
        replacement_data)->lastTouchTick = Tick(0);
}

void
FBRP::touch(const std::shared_ptr<ReplacementData>& replacement_data) const
{
    // Update reference count
    std::static_pointer_cast<FBReplData>(replacement_data)->refCount++;

    // Update last touch timestamp
    std::static_pointer_cast<FBReplData>(
        replacement_data)->lastTouchTick = curTick();
}

void
FBRP::reset(const std::shared_ptr<ReplacementData>& replacement_data) const
{
    // Reset reference count
    std::static_pointer_cast<FBReplData>(replacement_data)->refCount = 1;

    // Set last touch timestamp
    std::static_pointer_cast<FBReplData>(
        replacement_data)->lastTouchTick = curTick();
}

ReplaceableEntry*
FBRP::getVictim(const ReplacementCandidates& candidates) const
{
    // There must be at least one replacement candidate
    assert(candidates.size() > 0);

    // Visit all candidates to find victim
    ReplaceableEntry* victim = candidates[0];
    for (const auto& candidate : candidates) {
        // Update victim entry if necessary
        if (std::static_pointer_cast<FBReplData>(
                    candidate->replacementData)->refCount <
                std::static_pointer_cast<FBReplData>(
                    victim->replacementData)->refCount) {
            victim = candidate;
        }
	else if (std::static_pointer_cast<FBReplData>(
                    candidate->replacementData)->refCount ==
                std::static_pointer_cast<FBReplData>(
                    victim->replacementData)->refCount) {
            if (std::static_pointer_cast<FBReplData>(
                    candidate->replacementData)->lastTouchTick <
                std::static_pointer_cast<FBReplData>(
                    victim->replacementData)->lastTouchTick) {
            	victim = candidate;
            }
        }
    }

    return victim;
}

std::shared_ptr<ReplacementData>
FBRP::instantiateEntry()
{
    return std::shared_ptr<ReplacementData>(new FBReplData());
}

FBRP*
FBRPParams::create()
{
    return new FBRP(this);
}
```

Modify ReplacementPolicies.py:
```python
class FBRP(BaseReplacementPolicy):
    type = 'FBRP'
    cxx_class = 'FBRP'
    cxx_header = "mem/cache/replacement_policies/fb_rp.hh"
```

Modify SConscript:
```python
Source('fb_rp.cc')
```

Modify Caches.py (add the following line in L3Cache class):
```python
replacement_policy = Param.BaseReplacementPolicy(FBRP(),"Replacement policy")
```

測試:

```
program: quicksort
l3_assoc: 2-way, 4-way
l3_size: 128kB, 256kB, 512kB, 1MB (其餘的cache size follow benchmark規定)
```

<!--Command (take l3_size=1MB as an example):
- 2-way:
```shell
./build/X86/gem5.opt configs/example/se.py -c tests/test-progs/benchmark/quicksort --cpu-type=TimingSimpleCPU --caches --l2cache --l3cache --l3_assoc=2 --l1i_size=32kB --l1d_size=32kB --l2_size=128kB --l3_size=1MB --mem-type=NVMainMemory --nvmain-config=../NVmain/Config/PCM_ISSCC_2012_4GB.config > NVMain.log
```
- 4-way:
```shell
./build/X86/gem5.opt configs/example/se.py -c tests/test-progs/benchmark/quicksort --cpu-type=TimingSimpleCPU --caches --l2cache --l3cache --l3_assoc=4--l1i_size=32kB --l1d_size=32kB --l2_size=128kB --l3_size=1MB --mem-type=NVMainMemory --nvmain-config=../NVmain/Config/PCM_ISSCC_2012_4GB.config > NVMain.log
``` -->

Key points:
- FBRP is not merely LFU, it is like a mix of LFU and LRU.
- I just modified fb_rp.hh & fb_rp.cc based on lfu_rp.hh and lfu_rp.cc (add lastTouchTick in LRU into it & modify the replacement logic according to the slides)

Result & Analysis:
- 2-way時1MB看不出差異，其餘大小的結果皆為LRU的replacement次數較少
- 4-way時1MB皆沒有replacement，512kB及256kB時的結果皆為LRU的replacement次數較少，128kB時結果則為FBRP的replacement次數較少

### Q5
- Write back: default, just run.

- Write through: modify base.cc (add the following lines)
```c++
if (blk->isWritable()) {
	PacketPtr writeclean_pkt = writecleanBlk(blk, pkt->req->getDest(), pkt->id);
	writebacks.push_back(writeclean_pkt);
}
```

測試:

```
program: multiply
l3_assoc: 4-way
l3_size: 128kB, 1MB (其餘的cache size follow benchmark規定)
replacement policy: LRU, FBRP
```

Command (take l3_size=1MB as an example):
```shell
./build/X86/gem5.opt configs/example/se.py -c tests/test-progs/benchmark/multiply --cpu-type=TimingSimpleCPU --caches --l2cache --l3cache --l3_assoc=4 --l1i_size=32kB --l1d_size=32kB --l2_size=128kB --l3_size=1MB --mem-type=NVMainMemory --nvmain-config=../NVmain/Config/PCM_ISSCC_2012_4GB.config > NVMain.log
```

### Bonus
還沒做，目前的結果是測試LFU、FBRP、LRU分別用write back/write through三者的差異

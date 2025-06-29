# Modify write policy
cd project/gem5-525ce650e1a5bbe71c39d4b15598d6c003cc9f9e/src/mem/cache/base.cc

cd project/gem5-525ce650e1a5bbe71c39d4b15598d6c003cc9f9e

# Compile gem5
scons EXTRAS=../NVmain build/X86/gem5.opt

#--l3_size=1MB

./build/X86/gem5.opt configs/example/se.py -c tests/test-progs/benchmark/multiply --cpu-type=TimingSimpleCPU --caches --l2cache --l3cache --l3_assoc=4 --l1i_size=32kB --l1d_size=32kB --l2_size=128kB --l3_size=1MB --mem-type=NVMainMemory --nvmain-config=../NVmain/Config/PCM_ISSCC_2012_4GB.config > NVMain.log


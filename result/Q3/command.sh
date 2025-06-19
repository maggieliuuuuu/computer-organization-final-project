cd project/gem5-525ce650e1a5bbe71c39d4b15598d6c003cc9f9e

#--l3_size=1MB

# 2-way
./build/X86/gem5.opt configs/example/se.py -c tests/test-progs/benchmark/quicksort --cpu-type=TimingSimpleCPU --caches --l2cache --l3cache --l3_assoc=2 --l1i_size=32kB --l1d_size=32kB --l2_size=128kB --l3_size=1MB --mem-type=NVMainMemory --nvmain-config=../NVmain/Config/PCM_ISSCC_2012_4GB.config > NVMain.log

# full-way
./build/X86/gem5.opt configs/example/se.py -c tests/test-progs/benchmark/quicksort --cpu-type=TimingSimpleCPU --caches --l2cache --l3cache --l3_assoc=16384 --l1i_size=32kB --l1d_size=32kB --l2_size=128kB --l3_size=1MB --mem-type=NVMainMemory --nvmain-config=../NVmain/Config/PCM_ISSCC_2012_4GB.config > NVMain.log


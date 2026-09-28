"""튜닝 --scenario-workers 권장값 — 이 컴퓨터의 가용 메모리와 CPU로 계산한다(Windows·Linux).

  python -m control.arena_suggest_workers 2.0            (M17, 작업자 1개당 GiB)
  python -m control.arena_suggest_workers 1.2 --physical-cores 6

규칙(kj 2026-09-28): 작업자 수 ≤ (가용 메모리 × 0.7) / 작업자당 메모리.
추가로 CPU − 1(부모·시스템 몫)과 튜닝 시나리오 수(18)를 넘지 않는다. 논리 CPU는 하이퍼스레딩이면
물리 코어의 2배로 잡히는데, 무거운 단일 스레드 작업은 물리 코어 수를 넘기면 느려진다(맥 실측) —
--physical-cores를 주면 그 값을 CPU 수로 쓴다(Windows 작업 관리자 > 성능 > CPU의 '코어' 값).
"""
import argparse
import os
import sys

MEMORY_FRACTION = 0.7
TUNING_SCENARIOS = 18


def recommended_workers(available_gib, per_worker_gib, cpus, scenarios=TUNING_SCENARIOS):
    by_memory = int(available_gib*MEMORY_FRACTION // per_worker_gib)
    return max(1, min(by_memory, cpus - 1, scenarios))


def available_memory_gib():
    """지금 쓸 수 있는 물리 메모리 [GiB]. Windows: GlobalMemoryStatusEx, Linux: MemAvailable."""
    if os.name == 'nt':
        import ctypes

        class MemoryStatus(ctypes.Structure):
            _fields_ = [('dwLength', ctypes.c_ulong), ('dwMemoryLoad', ctypes.c_ulong),
                        ('ullTotalPhys', ctypes.c_ulonglong), ('ullAvailPhys', ctypes.c_ulonglong),
                        ('ullTotalPageFile', ctypes.c_ulonglong), ('ullAvailPageFile', ctypes.c_ulonglong),
                        ('ullTotalVirtual', ctypes.c_ulonglong), ('ullAvailVirtual', ctypes.c_ulonglong),
                        ('ullAvailExtendedVirtual', ctypes.c_ulonglong)]

        status = MemoryStatus()
        status.dwLength = ctypes.sizeof(MemoryStatus)
        if not ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(status)):
            raise OSError('GlobalMemoryStatusEx failed')
        return status.ullAvailPhys/2**30
    with open('/proc/meminfo', encoding='utf-8') as stream:
        for line in stream:
            if line.startswith('MemAvailable:'):
                return int(line.split()[1])/2**20
    raise OSError('MemAvailable not found in /proc/meminfo')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('per_worker_gib', type=float, help='memory per worker [GiB] (M17 2.0, F13 1.2, V13 0.6)')
    parser.add_argument('--physical-cores', type=int, help='use this instead of the logical CPU count')
    args = parser.parse_args(argv)
    available = available_memory_gib()
    logical = os.cpu_count() or 1
    cpus = args.physical_cores or logical
    n = recommended_workers(available, args.per_worker_gib, cpus)
    source = 'physical cores (given)' if args.physical_cores else 'logical CPUs'
    print(f'available memory {available:.1f} GiB x {MEMORY_FRACTION} / {args.per_worker_gib} GiB per worker; '
          f'{cpus} {source} (logical {logical}) -> --scenario-workers {n}')
    return 0


if __name__ == '__main__':
    sys.exit(main())

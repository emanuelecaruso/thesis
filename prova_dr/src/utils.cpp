#include "utils.h"
#include <sys/time.h>

namespace pr {
  //! returns the current time in milliseconds
  double getTime(){
    struct timeval tv;
    gettimeofday(&tv, 0);
    return 1e3*tv.tv_sec+tv.tv_usec*1e-3;
  }

  //! prints a null terminated array od strings
  void printBanner(const char** banner, std::ostream& os){
    while (*banner) {
      os << *banner << std::endl;
      banner++;
    }
  }

// Simple parallel for used since this program do not yet support
// parallel algorithms. `Func` takes the integer index.
template <typename Func>
void parallel_for(int begin, int end, Func&& func) {
  auto             futures  = std::vector<std::future<void>>{};
  auto             nthreads = std::thread::hardware_concurrency();
  std::atomic<int> next_idx(begin);
  for (auto thread_id = 0; thread_id < nthreads; thread_id++) {
    futures.emplace_back(
        std::async(std::launch::async, [&func, &next_idx, end]() {
          while (true) {
            auto idx = next_idx.fetch_add(1);
            if (idx >= end) break;
            func(idx);
          }
        }));
  }
  for (auto& f : futures) f.get();
}
}

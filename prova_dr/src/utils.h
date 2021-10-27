#pragma once
#include <iostream>
#include <future>
#include <thread>
#include <vector>

namespace pr {
  //! returns the current time in milliseconds
  double getTime();

  //! prints a null terminated array od strings
  void printBanner(const char** banner, std::ostream& os=std::cout);

  // Simple parallel for used since this program do not yet support
  // parallel algorithms. `Func` takes the integer index.
  template <typename Func>
  void parallel_for(int cols, int rows, Func& func);

}

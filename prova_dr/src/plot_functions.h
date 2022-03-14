#pragma once
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;


inline void showLine(std::vector<float> vec, const std::string& name){
  plt::plot(vec);
  plt::title(name); // set a title
  plt::legend();
  plt::show();
}

inline void saveLine(std::vector<float> vec, const std::string& name ){
  plt::plot(vec);
  // plt::savefig(name);

}

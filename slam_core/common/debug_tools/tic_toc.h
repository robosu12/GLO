/************************************************************************
 * Software License Agreement (BSD License)
 ************************************************************************/
#ifndef TICTOC_H_
#define TICTOC_H_

#include <iostream>
#include <string>
#include <ctime>
#include <cstdlib>
#include <chrono>

namespace GR_SLAM {

class TicToc {
 public:
  /**
   *TicToc
   *@brief
   *计时类
   *
   **/
  TicToc() { tic(); }

  /**
   *tic
   *@brief
   *当前时间
   *
   **/
  void tic() { start = std::chrono::system_clock::now(); }

  /**
   *toc
   *@brief
   *耗时计算
   *
   **/
  double toc() {
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    return elapsed_seconds.count() * 1000;
  }

  double getAveTime() {
    sum_time += toc();
    count++;
    return sum_time / count;
  }

  /**
   *print
   *@brief
   *耗时打印
   *param[in] info-要打印的信息
   *
   **/
  void print(const std::string &info) {
    std::cout << info + "  " << toc() << "  ms !!" << std::endl;
  }

 private:
  std::chrono::time_point<std::chrono::system_clock> start, end;
  ///< 起始和结束时间
  double sum_time = 0.0;
  int count = 0;

};  // end of class

}  // namespace GR_SLAM

#endif  // TICTOC_H_

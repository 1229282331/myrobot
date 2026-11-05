// Read-only model audit. All FK/Jacobian calculations use Orocos KDL.
// This executable has no network or robot command path.
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <cmath>
#include <iomanip>
#include <iostream>

int main(int argc, char** argv) {
  if (argc != 4) { std::cerr << "usage: kdl_model_probe URDF BASE TIP\n"; return 2; }
  KDL::Tree tree;
  KDL::Chain chain;
  if (!kdl_parser::treeFromFile(argv[1], tree) || !tree.getChain(argv[2], argv[3], chain)
      || chain.getNrOfJoints() != 6) return 3;
  KDL::ChainFkSolverPos_recursive fk(chain);
  KDL::ChainJntToJacSolver jacobian(chain);
  KDL::JntArray q(6);
  std::cout << std::setprecision(17);
  while (std::cin >> q(0)) {
    for (unsigned i=1; i<6; ++i) if (!(std::cin >> q(i))) return 4;
    for (unsigned i=0; i<6; ++i) if (!std::isfinite(q(i))) return 5;
    KDL::Frame frame;
    KDL::Jacobian jac(6);
    if (fk.JntToCart(q, frame) < 0 || jacobian.JntToJac(q, jac) < 0) return 6;
    double x,y,z,w;
    frame.M.GetQuaternion(x,y,z,w);
    std::cout << "{\"position_m\":[" << frame.p.x() << ',' << frame.p.y() << ',' << frame.p.z()
      << "],\"quaternion_xyzw\":[" << x << ',' << y << ',' << z << ',' << w << "],\"jacobian\":[";
    for (unsigned i=0; i<6; ++i) {
      if (i) std::cout << ',';
      std::cout << '[';
      for (unsigned j=0; j<6; ++j) { if (j) std::cout << ','; std::cout << jac(i,j); }
      std::cout << ']';
    }
    std::cout << "]}" << std::endl;
  }
}

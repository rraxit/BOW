//| Copyright Inria May 2015
//| This project has received funding from the European Research Council (ERC) under
//| the European Union's Horizon 2020 research and innovation programme (grant
//| agreement No 637972) - see http://www.resibots.eu
//|
//| Contributor(s):
//|   - Jean-Baptiste Mouret (jean-baptiste.mouret@inria.fr)
//|   - Antoine Cully (antoinecully@gmail.com)
//|   - Konstantinos Chatzilygeroudis (konstantinos.chatzilygeroudis@inria.fr)
//|   - Federico Allocati (fede.allocati@gmail.com)
//|   - Vaios Papaspyros (b.papaspyros@gmail.com)
//|   - Roberto Rama (bertoski@gmail.com)
//|
//| This software is a computer library whose purpose is to optimize continuous,
//| black-box functions. It mainly implements Gaussian processes and Bayesian
//| optimization.
//| Main repository: http://github.com/resibots/limbo
//| Documentation: http://www.resibots.eu/limbo
//|
//| This software is governed by the CeCILL-C license under French law and
//| abiding by the rules of distribution of free software.  You can  use,
//| modify and/ or redistribute the software under the terms of the CeCILL-C
//| license as circulated by CEA, CNRS and INRIA at the following URL
//| "http://www.cecill.info".
//|
//| As a counterpart to the access to the source code and  rights to copy,
//| modify and redistribute granted by the license, users are provided only
//| with a limited warranty  and the software's author,  the holder of the
//| economic rights,  and the successive licensors  have only  limited
//| liability.
//|
//| In this respect, the user's attention is drawn to the risks associated
//| with loading,  using,  modifying and/or developing or reproducing the
//| software by the user in light of its specific status of free software,
//| that may mean  that it is complicated to manipulate,  and  that  also
//| therefore means  that it is reserved for developers  and  experienced
//| professionals having in-depth computer knowledge. Users are therefore
//| encouraged to load and test_v0 the software's suitability as regards their
//| requirements in conditions enabling the security of their systems and/or
//| data to be ensured and,  more generally, to use and operate it in the
//| same conditions as regards security.
//|
//| The fact that you are presently reading this means that you have had
//| knowledge of the CeCILL-C license and that you accept its terms.
//|
#ifndef LIMBO_KERNEL_MATERN_THREE_HALVES_HPP
#define LIMBO_KERNEL_MATERN_THREE_HALVES_HPP

#include <limbo/kernel/kernel.hpp>

namespace limbo {
    namespace defaults {
        struct kernel_maternthreehalves {
            /// @ingroup kernel_defaults
            BO_PARAM(double, sigma_sq, 1);
            /// @ingroup kernel_defaults
            BO_PARAM(double, l, 1);
        };
    } // namespace defaults
    namespace kernel {
        /**
         @ingroup kernel
         \rst
         Matern 3/2 kernel

         .. math::
           d = ||v1 - v2||

           \nu = 3/2

           C(d) = \sigma^2\frac{2^{1-\nu}}{\Gamma(\nu)}\Bigg(\sqrt{2\nu}\frac{d}{l}\Bigg)^\nu K_\nu\Bigg(\sqrt{2\nu}\frac{d}{l}\Bigg),


         Parameters:
          - ``double sigma_sq`` (signal variance)
          - ``double l`` (characteristic length scale)

        Reference: :cite:`matern1960spatial` & :cite:`brochu2010tutorial` p.10 & https://en.wikipedia.org/wiki/Mat%C3%A9rn_covariance_function
        \endrst
        */
        template <typename Params>
        struct MaternThreeHalves : public BaseKernel<Params, MaternThreeHalves<Params>> {
            MaternThreeHalves(size_t dim = 1) : _sf2(Params::kernel_maternthreehalves::sigma_sq()), _l(Params::kernel_maternthreehalves::l())
            {
                _h_params = Eigen::VectorXd(2);
                _h_params << std::log(_l), std::log(std::sqrt(_sf2));
            }

            size_t params_size() const { return 2; }

            // Return the hyper parameters in log-space
            Eigen::VectorXd params() const { return _h_params; }

            // We expect the input parameters to be in log-space
            void set_params(const Eigen::VectorXd& p)
            {
                _h_params = p;
                _l = std::exp(p(0));
                _sf2 = std::exp(2.0 * p(1));
            }

            double kernel(const Eigen::VectorXd& v1, const Eigen::VectorXd& v2) const
            {
                double d = (v1 - v2).norm();
                double term = std::sqrt(3) * d / _l;

                return _sf2 * (1 + term) * std::exp(-term);
            }

            Eigen::VectorXd gradient(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2) const
            {
                Eigen::VectorXd grad(this->params_size());

                double d = (x1 - x2).norm();
                double term = std::sqrt(3) * d / _l;
                double r = std::exp(-term);

                // derivative of (1+term) = -term
                // derivative of e^(-term) = term*r
                grad(0) = _sf2 * (-term * r + (1 + term) * term * r);
                grad(1) = 2 * _sf2 * (1 + term) * r;

                return grad;
            }

        protected:
            double _sf2, _l;

            Eigen::VectorXd _h_params;
        };
    } // namespace kernel
} // namespace limbo

#endif

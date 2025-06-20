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
#include <limbo/limbo.hpp>

#include <limbo/experimental/bayes_opt/ehvi.hpp>
#include <limbo/experimental/bayes_opt/nsbo.hpp>
#include <limbo/experimental/bayes_opt/parego.hpp>
#include <limbo/experimental/stat/hyper_volume.hpp>
#include <limbo/experimental/stat/pareto_benchmark.hpp>
#include <limbo/experimental/stat/pareto_front.hpp>

using namespace limbo;

struct Params {
    struct bayes_opt_boptimizer : public defaults::bayes_opt_boptimizer {
    };

    struct init_randomsampling {
        BO_PARAM(int, samples, 10);
    };

    struct kernel : public defaults::kernel {
        BO_PARAM(double, noise, 0.01);
    };

    struct kernel_maternfivehalves : public defaults::kernel_maternfivehalves {
    };

    struct bayes_opt_bobase : public defaults::bayes_opt_bobase {
        BO_PARAM(bool, stats_enabled, true);
    };

    struct stop_maxiterations {
        BO_PARAM(int, iterations, 30);
    };

    struct acqui_ucb : public defaults::acqui_ucb {
    };

    struct acqui_gpucb : public defaults::acqui_gpucb {
    };

#ifdef USE_NLOPT
    struct opt_nloptnograd : public defaults::opt_nloptnograd {
    };
#elif defined(USE_LIBCMAES)
    struct opt_cmaes : public defaults::opt_cmaes {
    };
#else
    struct opt_gridsearch : public defaults::opt_gridsearch {
    };
#endif

    struct mean_constant : public defaults::mean_constant {
    };

    struct bayes_opt_ehvi : public defaults::bayes_opt_ehvi {
        BO_PARAM(double, x_ref, -11);
        BO_PARAM(double, y_ref, -11);
    };

    struct model_gp_parego : public experimental::defaults::model_gp_parego {
    };

    struct stat_hyper_volume {
        BO_PARAM_ARRAY(double, ref, 10, 10);
    };
};

#ifdef DIM6
#define ZDT_DIM 6
#elif defined(DIM2)
#define ZDT_DIM 2
#else
#define ZDT_DIM 30
#endif

struct zdt1 {
    BO_PARAM(size_t, dim_in, ZDT_DIM);
    BO_PARAM(size_t, dim_out, 2);

    Eigen::VectorXd operator()(const Eigen::VectorXd& x) const
    {
        Eigen::VectorXd res(2);
        double f1 = x(0);
        double g = 1.0;
        for (int i = 1; i < x.size(); ++i)
            g += 9.0 / (x.size() - 1) * x(i);
        double h = 1.0f - sqrtf(f1 / g);
        double f2 = g * h;
        res(0) = 1.0 - f1;
        res(1) = 1.0 - f2;
        return res;
    }
};

struct zdt2 {
    BO_PARAM(size_t, dim_in, ZDT_DIM);
    BO_PARAM(size_t, dim_out, 2);

    Eigen::VectorXd operator()(const Eigen::VectorXd& x) const
    {
        Eigen::VectorXd res(2);
        double f1 = x(0);
        double g = 1.0;
        for (int i = 1; i < x.size(); ++i)
            g += 9.0 / (x.size() - 1) * x(i);
        double h = 1.0f - pow((f1 / g), 2.0);
        double f2 = g * h;
        res(0) = 1.0 - f1;
        res(1) = 1.0 - f2;
        return res;
    }
};

struct zdt3 {
    BO_PARAM(size_t, dim_in, ZDT_DIM);
    BO_PARAM(size_t, dim_out, 2);

    Eigen::VectorXd operator()(const Eigen::VectorXd& x) const
    {
        Eigen::VectorXd res(2);
        double f1 = x(0);
        double g = 1.0;
        for (int i = 1; i < x.size(); ++i)
            g += 9.0 / (x.size() - 1) * x(i);
        double h = 1.0f - sqrtf(f1 / g) - f1 / g * sin(10 * M_PI * f1);
        double f2 = g * h;
        res(0) = 1.0 - f1;
        res(1) = 1.0 - f2;
        return res;
    }
};

struct mop2 {
    BO_PARAM(size_t, dim_in, 2);
    BO_PARAM(size_t, dim_out, 2);

    Eigen::VectorXd operator()(const Eigen::VectorXd& x) const
    {
        Eigen::VectorXd res(2);
        // scale to [-2, 2]
        Eigen::VectorXd xx = (x * 4.0).array() - 2.0;
        // f1, f2
        Eigen::VectorXd v1 = (xx.array() - 1.0 / sqrt(xx.size())).array().square();
        Eigen::VectorXd v2 = (xx.array() + 1.0 / sqrt(xx.size())).array().square();
        double f1 = 1.0 - exp(-v1.sum());
        double f2 = 1.0 - exp(-v2.sum());
        // we _maximize in [0:1]
        res(0) = 1 - f1;
        res(1) = 1 - f2;
        return res;
    }
};

int main()
{
    tools::par::init();

#ifdef ZDT1
    using func_t = zdt1;
#elif defined ZDT2
    using func_t = zdt2;
#elif defined ZDT3
    using func_t = zdt3;
#else
    using func_t = mop2;
#endif

#ifdef PAREGO
    using stat_t = boost::fusion::vector<experimental::stat::ParetoFront<Params>,
        experimental::stat::HyperVolume<Params>,
        stat::ConsoleSummary<Params>>;
    experimental::bayes_opt::Parego<Params, statsfun<stat_t>> opt;
#elif defined(NSBO)
    using stat_t = boost::fusion::vector<experimental::stat::ParetoBenchmark<func_t>>;
    experimental::bayes_opt::Nsbo<Params, statsfun<stat_t>> opt;
#else
    using stat_t = boost::fusion::vector<experimental::stat::ParetoBenchmark<func_t>>;
    experimental::bayes_opt::Ehvi<Params, statsfun<stat_t>> opt;
#endif
    opt.optimize(func_t());

    return 0;
}

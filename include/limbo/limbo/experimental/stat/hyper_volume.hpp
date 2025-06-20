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
#ifndef LIMBO_STAT_HYPER_VOLUME_HPP
#define LIMBO_STAT_HYPER_VOLUME_HPP

#include <hv/hypervol.h>
#include <limbo/experimental/tools/pareto.hpp>
#include <limbo/stat/stat_base.hpp>

namespace limbo {
    namespace experimental {
        namespace stat {
            namespace defaults {
                struct stat_hyper_volume {
                    BO_PARAM_ARRAY(double, ref, 10, 10);
                };
            }

            template <typename Params>
            struct HyperVolume : public limbo::stat::StatBase<Params> {
                template <typename BO, typename AggregatorFunction>
                void operator()(const BO& bo, const AggregatorFunction&)
                {
                    if (bo.observations().empty())
                        return;
                    if (!bo.stats_enabled())
                        return;
                    // convert the data to C arrays
                    double** data = new double*[bo.observations().size()];
                    for (size_t i = 0; i < bo.observations().size(); ++i) {
                        size_t dim = bo.observations()[i].size();
                        data[i] = new double[dim];
                        for (size_t k = 0; k < dim; ++k)
                            data[i][k] = bo.observations()[i](k) + Params::stat_hyper_volume::ref(k);
                    }
                    // call the hypervolume by Zitzler
                    int noObjectives = bo.observations()[0].size();
                    int redSizeFront1 = FilterNondominatedSet(data, bo.observations().size(), noObjectives);
                    double hv = CalculateHypervolume(data, redSizeFront1, noObjectives);

                    // write
                    this->_create_log_file(bo, "hypervolume.dat");
                    (*this->_log_file) << bo.current_iteration() << "\t" << hv << std::endl;

                    // free data
                    for (size_t i = 0; i < bo.observations().size(); ++i)
                        delete[] data[i];
                    delete[] data;
                }
            };
        }
    }
}

#endif

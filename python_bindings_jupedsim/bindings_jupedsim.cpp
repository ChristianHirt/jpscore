/// Copyright © 2012-2022 Forschungszentrum Jülich GmbH
/// SPDX-License-Identifier: LGPL-3.0-or-later
#include <jupedsim/jupedsim.h>
#include <jupedsim/jupedsim_experimental.h>

#include <algorithm>
#include <exception>
#include <iterator>
#include <memory>
#include <pybind11/detail/common.h>
#include <stdexcept>
#include <vector>

#include <fmt/format.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// TODO(kkratz):
//  * write python doc
//  * write python test that exercises all bindings
//  * double check python object ownership

namespace py = pybind11;

#define OWNED_WRAPPER(cls)                                                                         \
    struct cls##_Wrapper {                                                                         \
        cls handle;                                                                                \
        cls##_Wrapper(cls h) : handle(h) {}                                                        \
        ~cls##_Wrapper() { cls##_Free(handle); }                                                   \
        cls##_Wrapper(const cls##_Wrapper&) = delete;                                              \
        cls##_Wrapper& operator=(const cls##_Wrapper&) = delete;                                   \
        cls##_Wrapper(cls##_Wrapper&&) = delete;                                                   \
        cls##_Wrapper& operator=(cls##_Wrapper&&) = delete;                                        \
    }

#define WRAPPER(cls)                                                                               \
    struct cls##_Wrapper {                                                                         \
        cls handle;                                                                                \
        cls##_Wrapper(cls h) : handle(h) {}                                                        \
        ~cls##_Wrapper() = default;                                                                \
        cls##_Wrapper(const cls##_Wrapper&) = delete;                                              \
        cls##_Wrapper& operator=(const cls##_Wrapper&) = delete;                                   \
        cls##_Wrapper(cls##_Wrapper&&) = delete;                                                   \
        cls##_Wrapper& operator=(cls##_Wrapper&&) = delete;                                        \
    }

// Public types
OWNED_WRAPPER(JPS_Geometry);
OWNED_WRAPPER(JPS_GeometryBuilder);
OWNED_WRAPPER(JPS_OperationalModel);
OWNED_WRAPPER(JPS_VelocityModelBuilder);
OWNED_WRAPPER(JPS_GCFMModelBuilder);
OWNED_WRAPPER(JPS_Areas);
OWNED_WRAPPER(JPS_AreasBuilder);
OWNED_WRAPPER(JPS_Journey);
OWNED_WRAPPER(JPS_Simulation);
OWNED_WRAPPER(JPS_GCFMModelAgentIterator);
OWNED_WRAPPER(JPS_VelocityModelAgentIterator);

// Experimental only types
OWNED_WRAPPER(JPS_RoutingEngine);

class LogCallbackOwner
{
public:
    using LogCallback = std::function<void(const std::string&)>;

    LogCallback debug{};
    LogCallback info{};
    LogCallback warning{};
    LogCallback error{};

public:
    static LogCallbackOwner& Instance()
    {
        static LogCallbackOwner instance;
        return instance;
    }
};

PYBIND11_MODULE(py_jupedsim, m)
{
    auto atexit = py::module_::import("atexit");
    atexit.attr("register")(py::cpp_function([]() {
        auto& owner = LogCallbackOwner::Instance();
        owner.debug = {};
        owner.info = {};
        owner.warning = {};
        owner.error = {};
    }));
    m.doc() = "JuPedSim Python bindings";
    m.def("set_debug_callback", [](LogCallbackOwner::LogCallback callback) {
        LogCallbackOwner::Instance().debug = callback;
        JPS_Logging_SetDebugCallback(
            [](const char* msg, void*) { LogCallbackOwner::Instance().debug(msg); }, nullptr);
    });
    m.def("set_info_callback", [](LogCallbackOwner::LogCallback callback) {
        LogCallbackOwner::Instance().info = callback;
        JPS_Logging_SetInfoCallback(
            [](const char* msg, void*) { LogCallbackOwner::Instance().info(msg); }, nullptr);
    });
    m.def("set_warning_callback", [](LogCallbackOwner::LogCallback callback) {
        LogCallbackOwner::Instance().warning = callback;
        JPS_Logging_SetWarningCallback(
            [](const char* msg, void*) { LogCallbackOwner::Instance().warning(msg); }, nullptr);
    });
    m.def("set_error_callback", [](LogCallbackOwner::LogCallback callback) {
        LogCallbackOwner::Instance().error = callback;
        JPS_Logging_SetErrorCallback(
            [](const char* msg, void*) { LogCallbackOwner::Instance().error(msg); }, nullptr);
    });
    py::class_<JPS_Point>(m, "Point")
        .def(py::init())
        .def(py::init([](std::tuple<double, double> tup) {
            return JPS_Point{std::get<0>(tup), std::get<1>(tup)};
        }))
        .def_readwrite("x", &JPS_Point::x)
        .def_readwrite("y", &JPS_Point::y)
        .def(
            "__getitem__",
            [](const JPS_Point& pt, int idx) {
                switch(idx) {
                    case 0:
                        return pt.x;
                    case 1:
                        return pt.y;
                    default:
                        throw pybind11::index_error{};
                }
            })
        .def(
            "__setitem__",
            [](JPS_Point& pt, int idx, double value) {
                switch(idx) {
                    case 0:
                        pt.x = value;
                        break;
                    case 1:
                        pt.y = value;
                        break;
                    default:
                        throw pybind11::index_error{};
                }
            })
        .def(
            "__eq__",
            [](const JPS_Point& lhs, const JPS_Point& rhs) {
                return lhs.x == rhs.x && lhs.y == rhs.y;
            })
        .def(
            "__str__", [](const JPS_Point& pt) { return fmt::format("({:f}, {:f})", pt.x, pt.y); });

    py::implicitly_convertible<std::tuple<double, double>, JPS_Point>();
    py::implicitly_convertible<std::tuple<int, int>, JPS_Point>();
    py::class_<JPS_GCFMModelAgentParameters>(m, "GCFMModelAgentParameters")
        .def(py::init())
        .def_readwrite("speed", &JPS_GCFMModelAgentParameters::speed)
        .def_readwrite("e0", &JPS_GCFMModelAgentParameters::e0)
        .def_readwrite("position", &JPS_GCFMModelAgentParameters::position)
        .def_readwrite("orientation", &JPS_GCFMModelAgentParameters::orientation)
        .def_readwrite("journey_id", &JPS_GCFMModelAgentParameters::journeyId)
        .def_readwrite("profile_id", &JPS_GCFMModelAgentParameters::profileId)
        .def_readwrite("id", &JPS_GCFMModelAgentParameters::agentId);
    py::class_<JPS_VelocityModelAgentParameters>(m, "VelocityModelAgentParameters")
        .def(py::init())
        .def_readwrite("e0", &JPS_VelocityModelAgentParameters::e0)
        .def_readwrite("position", &JPS_VelocityModelAgentParameters::position)
        .def_readwrite("orientation", &JPS_VelocityModelAgentParameters::orientation)
        .def_readwrite("journey_id", &JPS_VelocityModelAgentParameters::journeyId)
        .def_readwrite("profile_id", &JPS_VelocityModelAgentParameters::profileId)
        .def_readwrite("id", &JPS_VelocityModelAgentParameters::agentId);
    py::class_<JPS_Geometry_Wrapper>(m, "Geometry");
    py::class_<JPS_GeometryBuilder_Wrapper>(m, "GeometryBuilder")
        .def(py::init([]() {
            return std::make_unique<JPS_GeometryBuilder_Wrapper>(JPS_GeometryBuilder_Create());
        }))
        .def(
            "add_accessible_area",
            [](const JPS_GeometryBuilder_Wrapper& w,
               std::vector<std::tuple<double, double>> points) {
                std::vector<double> values{};
                values.reserve(points.size() * 2);
                for(const auto [x, y] : points) {
                    values.emplace_back(x);
                    values.emplace_back(y);
                }
                JPS_GeometryBuilder_AddAccessibleArea(w.handle, values.data(), values.size() / 2);
            },
            "Add area where agents can move")
        .def(
            "exclude_from_accssible_area",
            [](const JPS_GeometryBuilder_Wrapper& w,
               std::vector<std::tuple<double, double>> points) {
                std::vector<double> values{};
                values.reserve(points.size() * 2);
                for(const auto [x, y] : points) {
                    values.emplace_back(x);
                    values.emplace_back(y);
                }
                JPS_GeometryBuilder_ExcludeFromAccessibleArea(
                    w.handle, values.data(), values.size() / 2);
            },
            "Add areas where agents can not move (obstacles)")
        .def(
            "build",
            [](const JPS_GeometryBuilder_Wrapper& w) {
                JPS_ErrorMessage errorMsg{};
                auto result = JPS_GeometryBuilder_Build(w.handle, &errorMsg);
                if(result) {
                    return std::make_unique<JPS_Geometry_Wrapper>(result);
                }
                auto msg = std::string(JPS_ErrorMessage_GetMessage(errorMsg));
                JPS_ErrorMessage_Free(errorMsg);
                throw std::runtime_error{msg};
            },
            "Geometry builder");
    py::class_<JPS_OperationalModel_Wrapper>(m, "OperationalModel");
    py::class_<JPS_VelocityModelBuilder_Wrapper>(m, "VelocityModelBuilder")
        .def(
            py::init([](double aPed, double DPed, double aWall, double DWall) {
                return std::make_unique<JPS_VelocityModelBuilder_Wrapper>(
                    JPS_VelocityModelBuilder_Create(aPed, DPed, aWall, DWall));
            }),
            py::kw_only(),
            py::arg("a_ped"),
            py::arg("d_ped"),
            py::arg("a_wall"),
            py::arg("d_wall"))
        .def(
            "add_parameter_profile",
            [](JPS_VelocityModelBuilder_Wrapper& w,
               JPS_ModelParameterProfileId id,
               double t,
               double tau,
               double v0,
               double radius) {
                JPS_VelocityModelBuilder_AddParameterProfile(w.handle, id, t, tau, v0, radius);
            },
            py::kw_only(),
            py::arg("id"),
            py::arg("time_gap"),
            py::arg("tau"),
            py::arg("v0"),
            py::arg("radius"))
        .def("build", [](JPS_VelocityModelBuilder_Wrapper& w) {
            JPS_ErrorMessage errorMsg{};
            auto result = JPS_VelocityModelBuilder_Build(w.handle, &errorMsg);
            if(result) {
                return std::make_unique<JPS_OperationalModel_Wrapper>(result);
            }
            auto msg = std::string(JPS_ErrorMessage_GetMessage(errorMsg));
            JPS_ErrorMessage_Free(errorMsg);
            throw std::runtime_error{msg};
        });
    py::class_<JPS_GCFMModelBuilder_Wrapper>(m, "GCFMModelBuilder")
        .def(
            py::init([](double nu_Ped,
                        double nu_Wall,
                        double dist_eff_Ped,
                        double dist_eff_Wall,
                        double intp_width_Ped,
                        double intp_width_Wall,
                        double maxf_Ped,
                        double maxf_Wall) {
                return std::make_unique<JPS_GCFMModelBuilder_Wrapper>(JPS_GCFMModelBuilder_Create(
                    nu_Ped,
                    nu_Wall,
                    dist_eff_Ped,
                    dist_eff_Wall,
                    intp_width_Ped,
                    intp_width_Wall,
                    maxf_Ped,
                    maxf_Wall));
            }),
            py::kw_only(),
            py::arg("nuPed"),
            py::arg("nuWall"),
            py::arg("distEffPed"),
            py::arg("distEffWall"),
            py::arg("intpWidthPed"),
            py::arg("intpWidthWall"),
            py::arg("maxfPed"),
            py::arg("maxfWall"))
        .def(
            "add_parameter_profile",
            [](JPS_GCFMModelBuilder_Wrapper& w,
               JPS_ModelParameterProfileId id,
               double mass,
               double tau,
               double v0,
               double a_v,
               double a_min,
               double b_min,
               double b_max) {
                JPS_GCFMModelBuilder_AddParameterProfile(
                    w.handle, id, mass, tau, v0, a_v, a_min, b_min, b_max);
            },
            py::kw_only(),
            py::arg("id"),
            py::arg("mass"),
            py::arg("tau"),
            py::arg("v0"),
            py::arg("a_v"),
            py::arg("a_min"),
            py::arg("b_min"),
            py::arg("b_max"))
        .def("build", [](JPS_GCFMModelBuilder_Wrapper& w) {
            JPS_ErrorMessage errorMsg{};
            auto result = JPS_GCFMModelBuilder_Build(w.handle, &errorMsg);
            if(result) {
                return std::make_unique<JPS_OperationalModel_Wrapper>(result);
            }
            auto msg = std::string(JPS_ErrorMessage_GetMessage(errorMsg));
            JPS_ErrorMessage_Free(errorMsg);
            throw std::runtime_error{msg};
        });

    py::class_<JPS_Areas_Wrapper>(m, "Areas");
    py::class_<JPS_AreasBuilder_Wrapper>(m, "AreasBuilder")
        .def(py::init(
            []() { return std::make_unique<JPS_AreasBuilder_Wrapper>(JPS_AreasBuilder_Create()); }))
        .def(
            "add_area",
            [](const JPS_AreasBuilder_Wrapper& w,
               uint64_t id,
               std::vector<std::tuple<double, double>> points,
               std::vector<std::string> tags) {
                std::vector<double> values{};
                values.reserve(points.size() * 2);
                for(const auto [x, y] : points) {
                    values.emplace_back(x);
                    values.emplace_back(y);
                }
                std::vector<const char*> tags_as_c_str;
                tags_as_c_str.reserve(tags.size());
                std::transform(
                    tags.begin(),
                    tags.end(),
                    std::back_inserter(tags_as_c_str),
                    [](const auto& str) { return str.c_str(); });

                JPS_AreasBuilder_AddArea(
                    w.handle,
                    id,
                    values.data(),
                    values.size() / 2,
                    tags_as_c_str.data(),
                    tags.size());
            },
            py::kw_only(),
            py::arg("id"),
            py::arg("polygon"),
            py::arg("labels"),
            "Add area")
        .def(
            "build",
            [](const JPS_AreasBuilder_Wrapper& w) {
                JPS_ErrorMessage errorMsg{};
                auto result = JPS_AreasBuilder_Build(w.handle, &errorMsg);
                if(result) {
                    return std::make_unique<JPS_Areas_Wrapper>(result);
                }
                auto msg = std::string(JPS_ErrorMessage_GetMessage(errorMsg));
                JPS_ErrorMessage_Free(errorMsg);
                throw std::runtime_error{msg};
            },
            "Build area");
    py::class_<JPS_Journey_Wrapper>(m, "Journey")
        .def_static(
            "make_waypoint_journey",
            [](const std::vector<std::tuple<std::tuple<double, double>, double>>& list) {
                std::vector<JPS_Waypoint> waypoints{};
                waypoints.reserve(list.size());
                for(const auto [pt, distance] : list) {
                    const auto [x, y] = pt;
                    waypoints.push_back(JPS_Waypoint{{x, y}, distance});
                }
                auto journey = JPS_Journey_Create_SimpleJourney(waypoints.data(), waypoints.size());
                return std::make_unique<JPS_Journey_Wrapper>(journey);
            });
    py::class_<JPS_GCFMModelAgentIterator_Wrapper>(m, "GCFMModelAgentIterator")
        .def(
            "__iter__",
            [](JPS_GCFMModelAgentIterator_Wrapper& w) -> JPS_GCFMModelAgentIterator_Wrapper& {
                return w;
            })
        .def("__next__", [](JPS_GCFMModelAgentIterator_Wrapper& w) {
            const auto result = JPS_GCFMModelAgentIterator_Next(w.handle);
            if(result) {
                return std::make_unique<JPS_GCFMModelAgentParameters>(*result);
            }
            throw py::stop_iteration{};
        });
    py::class_<JPS_VelocityModelAgentIterator_Wrapper>(m, "VelocityModelAgentIterator")
        .def(
            "__iter__",
            [](JPS_VelocityModelAgentIterator_Wrapper& w)
                -> JPS_VelocityModelAgentIterator_Wrapper& { return w; })
        .def("__next__", [](JPS_VelocityModelAgentIterator_Wrapper& w) {
            const auto result = JPS_VelocityModelAgentIterator_Next(w.handle);
            if(result) {
                return std::make_unique<JPS_VelocityModelAgentParameters>(*result);
            }
            throw py::stop_iteration{};
        });
    py::class_<JPS_Simulation_Wrapper>(m, "Simulation")
        .def(
            py::init([](JPS_OperationalModel_Wrapper& model,
                        JPS_Geometry_Wrapper& geometry,
                        JPS_Areas_Wrapper& areas,
                        double dT) {
                JPS_ErrorMessage errorMsg{};
                auto result = JPS_Simulation_Create(
                    model.handle, geometry.handle, areas.handle, dT, &errorMsg);
                if(result) {
                    return std::make_unique<JPS_Simulation_Wrapper>(result);
                }
                auto msg = std::string(JPS_ErrorMessage_GetMessage(errorMsg));
                JPS_ErrorMessage_Free(errorMsg);
                throw std::runtime_error{msg};
            }),
            py::kw_only(),
            py::arg("model"),
            py::arg("geometry"),
            py::arg("areas"),
            py::arg("dt"))
        .def(
            "add_journey",
            [](JPS_Simulation_Wrapper& simulation, JPS_Journey_Wrapper& journey) {
                JPS_ErrorMessage errorMsg{};
                const auto result =
                    JPS_Simulation_AddJourney(simulation.handle, journey.handle, &errorMsg);
                if(result != 0) {
                    return result;
                }
                auto msg = std::string(JPS_ErrorMessage_GetMessage(errorMsg));
                JPS_ErrorMessage_Free(errorMsg);
                throw std::runtime_error{msg};
            })
        .def(
            "add_agent",
            [](JPS_Simulation_Wrapper& simulation, JPS_GCFMModelAgentParameters& parameters) {
                JPS_ErrorMessage errorMsg{};
                auto result =
                    JPS_Simulation_AddGCFMModelAgent(simulation.handle, parameters, &errorMsg);
                if(result) {
                    return result;
                }
                auto msg = std::string(JPS_ErrorMessage_GetMessage(errorMsg));
                JPS_ErrorMessage_Free(errorMsg);
                throw std::runtime_error{msg};
            })
        .def(
            "add_agent",
            [](JPS_Simulation_Wrapper& simulation, JPS_VelocityModelAgentParameters& parameters) {
                JPS_ErrorMessage errorMsg{};
                auto result =
                    JPS_Simulation_AddVelocityModelAgent(simulation.handle, parameters, &errorMsg);
                if(result) {
                    return result;
                }
                auto msg = std::string(JPS_ErrorMessage_GetMessage(errorMsg));
                JPS_ErrorMessage_Free(errorMsg);
                throw std::runtime_error{msg};
            })
        .def(
            "remove_agent",
            [](JPS_Simulation_Wrapper& simulation, JPS_AgentId id) {
                JPS_ErrorMessage errorMsg{};
                auto result = JPS_Simulation_RemoveAgent(simulation.handle, id, &errorMsg);
                if(result) {
                    return result;
                }
                auto msg = std::string(JPS_ErrorMessage_GetMessage(errorMsg));
                JPS_ErrorMessage_Free(errorMsg);
                throw std::runtime_error{msg};
            })
        .def(
            "read_agent",
            [](JPS_Simulation_Wrapper& simulation, JPS_AgentId id) {
                JPS_ErrorMessage errorMsg{};
                using AgentVariants =
                    std::variant<JPS_GCFMModelAgentParameters, JPS_VelocityModelAgentParameters>;
                const auto type = JPS_Simulation_ModelType(simulation.handle);
                switch(type) {
                    case JPS_GCFMModel: {
                        JPS_GCFMModelAgentParameters agent{};
                        bool result = JPS_Simulation_ReadGCFMModelAgent(
                            simulation.handle, id, &agent, &errorMsg);
                        if(result) {
                            return AgentVariants{agent};
                        }
                        auto msg = std::string(JPS_ErrorMessage_GetMessage(errorMsg));
                        JPS_ErrorMessage_Free(errorMsg);
                        throw std::runtime_error{msg};
                        break;
                    }
                    case JPS_VelocityModel: {
                        JPS_VelocityModelAgentParameters agent{};
                        bool result = JPS_Simulation_ReadVelocityModelAgent(
                            simulation.handle, id, &agent, &errorMsg);
                        if(result) {
                            return AgentVariants{agent};
                        }
                        auto msg = std::string(JPS_ErrorMessage_GetMessage(errorMsg));
                        JPS_ErrorMessage_Free(errorMsg);
                        throw std::runtime_error{msg};
                        break;
                    }
                }
            })
        .def(
            "removed_agents",
            [](const JPS_Simulation_Wrapper& simulation) {
                const JPS_AgentId* ids{};
                const auto count = JPS_Simulation_RemovedAgents(simulation.handle, &ids);
                return std::vector<JPS_AgentId>{ids, ids + count};
            })
        .def(
            "iterate",
            [](const JPS_Simulation_Wrapper& simulation) {
                JPS_ErrorMessage errorMsg{};
                auto result = JPS_Simulation_Iterate(simulation.handle, &errorMsg);
                if(result) {
                    return;
                }
                auto msg = std::string(JPS_ErrorMessage_GetMessage(errorMsg));
                JPS_ErrorMessage_Free(errorMsg);
                throw std::runtime_error{msg};
            })
        .def(
            "switch_agent_profile",
            [](const JPS_Simulation_Wrapper& w,
               JPS_AgentId agentId,
               JPS_ModelParameterProfileId profileId) {
                JPS_ErrorMessage errorMsg{};
                auto result =
                    JPS_Simulation_SwitchAgentProfile(w.handle, agentId, profileId, &errorMsg);
                if(result) {
                    return;
                }
                auto msg = std::string(JPS_ErrorMessage_GetMessage(errorMsg));
                JPS_ErrorMessage_Free(errorMsg);
                throw std::runtime_error{msg};
            },
            py::kw_only(),
            py::arg("agent_id"),
            py::arg("profile_id"))
        .def(
            "agent_count",
            [](JPS_Simulation_Wrapper& simulation) {
                return JPS_Simulation_AgentCount(simulation.handle);
            })
        .def(
            "iteration_count",
            [](JPS_Simulation_Wrapper& simulation) {
                return JPS_Simulation_IterationCount(simulation.handle);
            })
        .def("agents", [](const JPS_Simulation_Wrapper& simulation) {
            using Iterators = std::variant<
                std::unique_ptr<JPS_GCFMModelAgentIterator_Wrapper>,
                std::unique_ptr<JPS_VelocityModelAgentIterator_Wrapper>>;
            const auto type = JPS_Simulation_ModelType(simulation.handle);
            switch(type) {
                case JPS_GCFMModel:
                    return Iterators{std::make_unique<JPS_GCFMModelAgentIterator_Wrapper>(
                        JPS_Simulation_GCFMModelAgentIterator(simulation.handle))};
                case JPS_VelocityModel:
                    return Iterators{std::make_unique<JPS_VelocityModelAgentIterator_Wrapper>(
                        JPS_Simulation_VelocityModelAgentIterator(simulation.handle))};
            }
        });
    py::module_ exp = m.def_submodule("experimental", "Experimental API extensions for jupedsim");
    py::class_<JPS_RoutingEngine_Wrapper>(exp, "RoutingEngine")
        .def(py::init([](const JPS_Geometry_Wrapper& geo) {
            return std::make_unique<JPS_RoutingEngine_Wrapper>(
                JPS_RoutingEngine_Create(geo.handle));
        }))
        .def(
            "compute_waypoints",
            [](const JPS_RoutingEngine_Wrapper& w,
               std::tuple<double, double> from,
               std::tuple<double, double> to) {
                auto intoJPS_Point = [](const auto p) {
                    return JPS_Point{std::get<0>(p), std::get<1>(p)};
                };
                auto waypoints = JPS_RoutingEngine_ComputeWaypoint(
                    w.handle, intoJPS_Point(from), intoJPS_Point(to));
                std::vector<std::tuple<double, double>> result;
                result.reserve(waypoints.len);
                std::transform(
                    waypoints.points,
                    waypoints.points + waypoints.len,
                    std::back_inserter(result),
                    [](const auto& p) { return std::make_tuple(p.x, p.y); });
                JPS_Path_Free(&waypoints);
                return result;
            })
        .def(
            "is_routable",
            [](const JPS_RoutingEngine_Wrapper& w, std::tuple<double, double> p) {
                return JPS_RoutingEngine_IsRoutable(
                    w.handle, JPS_Point{std::get<0>(p), std::get<1>(p)});
            })
        .def(
            "mesh",
            [](const JPS_RoutingEngine_Wrapper& w) {
                auto result = JPS_RoutingEngine_Mesh(w.handle);
                using Pt = std::tuple<double, double>;
                using Tri = std::tuple<Pt, Pt, Pt>;
                std::vector<Tri> mesh{};
                mesh.reserve(result.len);
                std::transform(
                    result.triangles,
                    result.triangles + result.len,
                    std::back_inserter(mesh),
                    [](const auto& t) {
                        return std::make_tuple(
                            std::make_tuple(t.points[0].x, t.points[0].y),
                            std::make_tuple(t.points[1].x, t.points[1].y),
                            std::make_tuple(t.points[2].x, t.points[2].y));
                    });
                JPS_TriangleMesh_Free(&result);
                return mesh;
            })
        .def("edges_for", [](const JPS_RoutingEngine_Wrapper& w, uint32_t id) {
            auto res = JPS_RoutingEngine_EdgesFor(w.handle, id);
            using Pt = std::tuple<double, double>;
            using Line = std::tuple<Pt, Pt>;
            std::vector<Line> lines{};
            lines.reserve(res.len);
            std::transform(
                res.lines, res.lines + res.len, std::back_inserter(lines), [](const auto& l) {
                    return std::make_tuple(
                        std::make_tuple(l.points[0].x, l.points[0].y),
                        std::make_tuple(l.points[1].x, l.points[1].y));
                });
            JPS_Lines_Free(&res);
            return lines;
        });
}

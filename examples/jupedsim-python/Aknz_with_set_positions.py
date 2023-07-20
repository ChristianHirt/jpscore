#! /usr/bin/env python3
import logging
import pathlib
import sys
import random

import py_jupedsim as jps
from jupedsim.trajectory_writer_sqlite import SqliteTrajectoryWriter
from jupedsim.util import build_jps_geometry
from jupedsim.distributions import distribute_by_number
from shapely import GeometryCollection, Polygon, geometry, to_wkt, from_wkt
from shapely.geometry.base import geom_factory

agents_left = [
(455.1659063720158, 1771.2995207272102)
]
agents_top = [
(517.25277244546, 1768.3506371600165)
]
agents_bottom = [
(517.0236902943427, 1761.3684009976496),
(515.3101469686682, 1762.0309766083776),
(504.80045542350376, 1766.5664875072575),
(488.6358271846974, 1762.1784958369358),
(484.8421037908732, 1761.796145675732),
(513.8216752070856, 1763.8291411511361),
(517.0985245368682, 1766.069175183802),
(483.48396137271493, 1761.6108730668561),
(496.7727679891885, 1766.8325017997176),
(497.41494083514914, 1762.7958912189147),
(504.0096006067818, 1762.3257349070177),
(506.1301515612452, 1762.3710039195216),
(490.8269702505534, 1765.0673779775223),
(507.439931670462, 1765.3680377931912),
(507.99740586190086, 1761.8590316169905),
(506.29119105361747, 1764.1572604297005),
(490.0701515264103, 1765.8721977300684),
(504.4283442740886, 1763.6098978096938),
(510.9244493711577, 1764.1516093856233),
(498.9603630634956, 1766.3158215880662),
(491.9799031438716, 1761.9946702231916),
(493.1917843127411, 1761.4245773929845),
(484.04061889110545, 1767.5769975704295),
(500.9903232583345, 1761.4933168709683),
(489.5536827444802, 1763.8954892173783),
(497.30647909462925, 1763.4671368361433),
(510.81303426891697, 1761.7496431533696),
(510.00477534969343, 1762.1817161994236),
(492.612209978593, 1765.625754115677),
(512.0310562772745, 1761.1595304213497),
(510.115185779789, 1762.499478007327),
(511.2301713599454, 1760.7455752186302),
(503.9213317139435, 1761.854033542929),
(503.081381152008, 1761.6194922013563),
(489.1934304584578, 1765.8838617254291),
(486.5210818284056, 1761.5800059705155),
(483.7530580410076, 1765.8632466166666),
(499.82929752739057, 1764.6235225422247),
(507.80085972403185, 1764.8570851174682),
(497.54681682783087, 1764.946284543945),
(514.9040129500934, 1761.5702164040881),
(497.1362115224031, 1764.7980418485115),
(499.8414656217662, 1763.9207832276095),
(505.0098128718243, 1760.6589213364618),
(504.5139650325781, 1767.0377775725326),
(495.3804065562379, 1763.779326511075),
(508.0537642008958, 1763.1954042689),
(504.1441858601805, 1766.4760990390407),
(489.0801707803393, 1767.4574632236536),
(486.21599292276414, 1767.6423185326412)
]


def log_debug(msg):
    logging.debug(msg)


def log_info(msg):
    logging.info(msg)


def log_warn(msg):
    logging.warning(msg)


def log_error(msg):
    logging.error(msg)


def main():
    logging.basicConfig(
        level=logging.DEBUG, format="%(levelname)s : %(message)s"
    )
    jps.set_debug_callback(log_debug)
    jps.set_info_callback(log_info)
    jps.set_warning_callback(log_warn)
    jps.set_error_callback(log_error)

    # get polygon from file
    with open("aknz_evac.wkt", 'r') as file:
        geometry_collection_as_string = file.readline()

    area = from_wkt(geometry_collection_as_string)
    geometry = build_jps_geometry(area)

    # build agent model
    model_builder = jps.VelocityModelBuilder(
        a_ped=8, d_ped=0.1, a_wall=5, d_wall=0.02
    )
    profile_id = 3
    model_builder.add_parameter_profile(
        id=profile_id, time_gap=1, tau=0.5, v0=1.2, radius=0.3
    )

    model = model_builder.build()

    simulation = jps.Simulation(model=model, geometry=geometry, dt=0.01)

    # create journeys
    journeys = []
    # journey from left concert area to top
    journey1 = jps.JourneyDescription()
    journey1.add_waypoint((428, 1788), 4.0)
    # upper left (top road) exit
    journey1.add_exit([(726.42, 1948.03), (721.32, 1964), (739.34, 1961.39), (746.12, 1959.66), (742.27, 1944.6)])
    journeys.append(simulation.add_journey(journey1))

    # journey from middle_top concert area to top
    journey2 = jps.JourneyDescription()
    journey2.add_waypoint((598.83, 1770.22), 1.0)
    # upper left (top road) exit
    journey2.add_exit([(726.42, 1948.03), (721.32, 1964), (739.34, 1961.39), (746.12, 1959.66), (742.27, 1944.6)])
    journeys.append(simulation.add_journey(journey2))

    # journey from middle_bottom to upper right exit over the top path
    journey3 = jps.JourneyDescription()
    journey3.add_waypoint((597.02, 1759.07), 1.0)
    journey3.add_waypoint((727.1, 1732.05), 1.0)
    # upper right (road_east) exit
    journey3.add_exit([(895.93, 1925.66), (896.41, 1944.51), (901.41, 1944.38), (900.91, 1924.94)])
    journeys.append(simulation.add_journey(journey3))
    # journey from middle_bottom to upper right exit over the bottom path
    journey4 = jps.JourneyDescription()
    journey4.add_waypoint((597.02, 1759.07), 1.0)
    journey4.add_waypoint((691.37, 1710.57), 1.0)
    # upper right (road_east) exit
    journey4.add_exit([(895.93, 1925.66), (896.41, 1944.51), (901.41, 1944.38), (900.91, 1924.94)])
    journeys.append(simulation.add_journey(journey4))

    agent_parameters = []
    for journey in journeys:
        agent_parameters.append(jps.VelocityModelAgentParameters())
        agent_parameters[-1].journey_id = journey
        agent_parameters[-1].orientation = (1.0, 0.0)
        agent_parameters[-1].position = (0.0, 0.0)
        agent_parameters[-1].profile_id = profile_id

    # distribute agents
    # concert_area_left
    s_polygon = Polygon([(446.4, 1778.95), (453.79, 1785.4), (460.9, 1787.88), (482.74, 1787.68),
                         (482.4, 1760.68), (464.59, 1761.22), (460.1, 1764.37), (446.4, 1766.0)])
    origin_seed = 1111426
    random.seed(origin_seed)
    seed = random.randint(10000, 99999)

    for agent_pos in agents_left:
        agent_parameters[0].position = agent_pos
        simulation.add_agent(agent_parameters[0])

    # concert_area_middle_top
    s_polygon = Polygon([(482.74, 1787.68), (499.55, 1787.7), (517.64, 1775.62), (517.84, 1768.0), (483, 1768.0)])
    seed = random.randint(10000, 99999)

    for agent_pos in agents_top:
        agent_parameters[1].position = agent_pos
        simulation.add_agent(agent_parameters[1])

    # concert_area_middle_bottom
    s_polygon = Polygon([(517.84, 1768), (517.84, 1759.96), (482.4, 1760.68), (483, 1768)])
    seed = random.randint(10000, 99999)
   
    for agent_pos in agents_bottom:
        random_number = random.random()
        if random_number > 0.5:
            agent_parameters[2].position = agent_pos
            simulation.add_agent(agent_parameters[2])
        else:
            agent_parameters[3].position = agent_pos
            simulation.add_agent(agent_parameters[3])

    print("Running simulation")

    writer = SqliteTrajectoryWriter(pathlib.Path("aknz_example_out.sqlite"))
    writer.begin_writing(10, to_wkt(area, rounding_precision=-1))

    while simulation.agent_count() > 0:
        try:
            simulation.iterate()
            if simulation.iteration_count() % 4 == 0:
                writer.write_iteration_state(simulation)

            if simulation.iteration_count() % 40:
                iteration = simulation.iteration_count()

                duration = simulation.get_last_trace().iteration_duration
                op_dur = simulation.get_last_trace().operational_level_duration

                print(
                    f"S-Time: {iteration / 100:6.2f}s "
                    f"I: {iteration:6d} "
                    f"Agents: {simulation.agent_count():4d} "
                    f"ItTime: {duration / 1000:6.2f}ms "
                    f"[OpLvl {op_dur / 1000:6.2f}ms]",
                    end="\r",
                )
        except KeyboardInterrupt:
            writer.end_writing()
            print("CTRL-C Received! Shutting down")
            sys.exit(1)
    writer.end_writing()
    print(
        f"Simulation completed after {simulation.iteration_count()} iterations"
    )


if __name__ == "__main__":
    main()

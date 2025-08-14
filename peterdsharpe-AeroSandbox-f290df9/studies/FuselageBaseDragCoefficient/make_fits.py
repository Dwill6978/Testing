from aerosandbox.tools.webplotdigitizer_reader import read_webplotdigitizer_csv
import aerosandbox as asb
import aerosandbox.numpy as np

data = read_webplotdigitizer_csv("mach_vs_base_drag_coefficient.csv")["data"]

import matplotlib.pyplot as plt
import aerosandbox.tools.pretty_plots as p

fig, ax = plt.subplots()
plt.plot(data[:, 0], data[:, 1], ".", label="Experimental Data")

plt.annotate(
    text='Wind tunnel data from MIL-HDBK-762,\n"Design of Aerodynamically Stabilized Free Rockets".\nFigure 5-138.',
    xy=(0.02, 0.02),
    xycoords="axes fraction",
    ha="left",
    fontsize=9,
    color="gray"
)


def model(m, p):
    return np.blend(
        p["trans_str"] * (m - p["m_trans"]),
        p["pc_sup"] + p["a"] * np.exp(-(p["scale_sup"] * (m - p["center_sup"])) ** 2),
        p["pc_sub"]
    )


fit = asb.FittedModel(
    model=model,
    x_data=data[:, 0],
    y_data=data[:, 1],
    parameter_guesses={
        "a"         : 0.2,
        "scale_sup" : 1,
        "center_sup": 1,
        "m_trans"   : 1.,
        "trans_str" : 5,
        "pc_sub"    : 0.16,
        "pc_sup"    : 0.05,
    },
    parameter_bounds={
        "trans_str": (0, 10),
    },
    verbose=False
)

# def model(m, p):
#     beta_squared_ideal = 1 - m ** 2
#
#     beta_squared = np.softmax(
#         beta_squared_ideal,
#         -beta_squared_ideal,
#         hardness=p["pg_hardness"]
#     )
#
#     return p["cd_0"] * beta_squared ** p["pg_power"]
#
#
# fit = asb.FittedModel(
#     model=model,
#     x_data=data[:, 0],
#     y_data=data[:, 1],
#     parameter_guesses={
#         "cd_0": 0.16,
#         "pg_hardness": 3,
#         "pg_power": -0.5,
#         "cd_offset": 0,
#     },
#     parameter_bounds={
#         # "pg_hardness": (1, None),
#         "pg_power": (None, 0),
#         # "cd_offset": (-1, 1),
#     },
#     # residual_norm_type="L1",
#     verbose=False
# )

from pprint import pprint

pprint(fit.parameters)

mach = np.linspace(0, 5, 1000)
plt.plot(mach, fit(mach), "-k", label="Fitted Model\nfor AeroBuildup")
plt.ylim(0, 0.2)

p.set_ticks(0.5, 0.1, 0.05, 0.01)

p.show_plot(
    "Fuselage Base Drag Coefficient",
    "Mach [-]",
    "Fuselage Base Drag Coefficient [-]"
)

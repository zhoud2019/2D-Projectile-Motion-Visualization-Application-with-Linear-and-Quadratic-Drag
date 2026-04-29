import tkinter as tk
import math

main_window = tk.Tk()
main_window.title("2D Projectile Simulator")
main_window.state('zoomed')

g = 9.81  # gravity constant

def enter_values_button():
    # Get inputs
    v0 = float(initial_velocity_entry.get())
    angle_deg = float(initial_angle_entry.get())
    mass = float(initial_mass_entry.get())
    alpha = math.radians(angle_deg)

    # Ideal trajectory calculation:
    def ideal_position(t):
        x = v0 * math.cos(alpha) * t
        y = v0 * math.sin(alpha) * t - 0.5 * g * t ** 2
        return x, y

    ideal_flight_time = (2 * v0 * math.sin(alpha)) / g
    ideal_max_height = (v0 ** 2 * (math.sin(alpha)) ** 2) / (2 * g)
    ideal_range = (v0 ** 2 * math.sin(2 * alpha)) / g

    # Linear drag
    D = 0.25  # diameter in meters
    C_quad = 0.22 * D ** 2 / mass
    k = C_quad * v0  # beta assumed = 0.5

    ascent_time_linear_drag = (1.0 / k) * math.log(1.0 + (k * v0 * math.sin(alpha)) / g)

    def y_ascent_linear_drag(t): #ascent time linear drag
        return -(g * t) / k + ((g + k * v0 * math.sin(alpha)) / k ** 2) * (1 - math.exp(-k * t))

    y_max_linear_drag = y_ascent_linear_drag(ascent_time_linear_drag)

    def gamma(k, t): #gamma function
        if t == 0:
            return 0.5
        return (math.exp(-k * t) - 1 + k * t) / (k ** 2 * t ** 2)

    descent_time_linear_drag = ((2 * v0 * math.sin(alpha)) / g) / 2 #initial guess for descent time

    for i in range(100): #100 iterations of the gamma function
        g_val = gamma(k, descent_time_linear_drag)

        t2_new = math.sqrt(y_max_linear_drag / (g * g_val))

        descent_time_linear_drag = t2_new

    t_total_linear_drag = ascent_time_linear_drag + descent_time_linear_drag #total flight time

    def x_pos_linear_drag(t): #x position linear drag
        return (v0 * math.cos(alpha) / k) * (1 - math.exp(-k * t))

    x_range_linear = x_pos_linear_drag(t_total_linear_drag)

    def y_descent_linear_drag(t): #y position linear drag
        return (g * t) / k - (g / k ** 2) * (1 - math.exp(-k * t))

    # Quadratic drag using LAT approximation
    # C = 0.22 * D^2 / m, assume D = 0.1m (diameter), so C = 0.22 * 0.01 / mass

    vx0 = v0 * math.cos(alpha) #initial x velocity
    vy0 = v0 * math.sin(alpha) #initail y velocity

    def x_pos_quad(t): #x position quadratic drag
        return (1.0 / C_quad) * math.log(C_quad * vx0 * t + 1)

    a_quad = C_quad * vx0  #acceleration quadratic drag
    w0 = vy0
    b = C_quad

    if angle_deg < 30:
        ascent_time_quad = (1 / math.sqrt(C_quad * g)) * math.atan(math.sqrt(C_quad / g) * vy0)

        def y_pos_quad(t):
            ln = math.log(1 + a_quad * t)
            return (w0 + g / (2 * a_quad)) * (1.0 / a_quad) * ln - 0.25 * g * t ** 2 - (g * t) / (2 * a_quad)

    elif 30 <= angle_deg < 60:  # SAT
        b_sat = math.sqrt(2 * b)
        omega_sat = math.sqrt(b_sat * g)
        phi_sat = math.atan(math.sqrt(b_sat / g) * vy0)
        ascent_time_quad = phi_sat / omega_sat  # defined right after its dependencies

        def y_pos_quad(t):
            if t <= ascent_time_quad:
                arg = max(math.cos(phi_sat - omega_sat * t) / math.cos(phi_sat), 1e-10)
                return (1.0 / b_sat) * math.log(arg)
            else:
                tau = t - ascent_time_quad
                w0_desc = math.sqrt(1 + b_sat * vy0 ** 2 / g)
                return (1.0 / b_sat) * (
                        math.log((2 * w0_desc) / (1 + math.exp(-2 * omega_sat * tau))) - omega_sat * tau)

    else:  # HAT
        omega_hat = math.sqrt(b * g)
        phi_hat = math.atan(math.sqrt(b / g) * vy0)
        ascent_time_quad = (1 / math.sqrt(C_quad * g)) * math.atan(math.sqrt(C_quad / g) * vy0)

        def y_pos_quad(t):
            if t <= ascent_time_quad:
                return (1.0 / b) * math.log(math.cos(phi_hat - omega_hat * t) / math.cos(phi_hat))
            else:
                tau = t - ascent_time_quad
                w0_desc = math.sqrt(1 + b * vy0 ** 2 / g)
                return (1.0 / b) * (math.log((2 * w0_desc) / (1 + math.exp(-2 * omega_hat * tau))) - omega_hat * tau)

    y_max_quad = y_pos_quad(ascent_time_quad)

    y_max_quad = y_pos_quad(ascent_time_quad)

    def vy_quad(t):
        return math.sqrt(g / C_quad) * math.tan(
            -math.sqrt(C_quad * g) * t + math.atan(math.sqrt(C_quad / g) * vy0))

    y_max_quad = y_pos_quad(ascent_time_quad)

    # Find total flight time for quadratic drag (when y = 0 again)
    t_lo2, t_hi2 = ascent_time_quad, ascent_time_quad + 0.1 #Setting the bounds, time interval that contains the landing moment
    while y_pos_quad(t_hi2) > 0:
        t_hi2 += 0.1 #Expands the time until y hits negative
    for j in range(200):
        t_mid2 = (t_lo2 + t_hi2) / 2 #take middle as a test point
        if y_pos_quad(t_mid2) > 0: #if y is positive at the midpoint, then the ball is still in the air. Therefore, there still has to be time
            t_lo2 = t_mid2         #before it lands onto the ground. Therefore, t_lo2 gets raised.
        else:
            t_hi2 = t_mid2 #Else, the ball has landed.

    t_total_quad = (t_lo2 + t_hi2) / 2
    x_range_quad = x_pos_quad(t_total_quad)

    main_window.update()
    W = main_window.winfo_width()
    H = main_window.winfo_height()

    canvas = tk.Canvas(main_window, width=W, height=H, bg="white")
    canvas.place(x=0, y=0)

    origin_x = 80
    origin_y = H - 100

    # Draw axes
    canvas.create_line(origin_x, origin_y, W - 60, origin_y, width=4)
    canvas.create_line(origin_x, origin_y, origin_x, 40, width=4)

    canvas.create_text(W - 80, origin_y + 30, text="meters", font=("Arial", 14))
    canvas.create_text(origin_x - 20, 25, text="meters", font=("Arial", 14))

    # Scale pixels per meter
    x_pixels = W - origin_x - 80
    y_pixels = origin_y - 40

    max_range = max(ideal_range, x_range_linear, x_range_quad, 1)
    max_height = max(ideal_max_height, y_max_linear_drag, y_max_quad, 1)

    pixels_x = x_pixels / max_range
    pixels_y = y_pixels / max_height

    # Ball creation
    ball_r = 10

    ideal_ball = canvas.create_oval(
        origin_x - ball_r, origin_y - ball_r,
        origin_x + ball_r, origin_y + ball_r,
        fill="blue"
    )

    linear_ball = canvas.create_oval(
        origin_x - ball_r, origin_y - ball_r,
        origin_x + ball_r, origin_y + ball_r,
        fill="red"
    )

    quad_ball = canvas.create_oval(
        origin_x - ball_r, origin_y - ball_r,
        origin_x + ball_r, origin_y + ball_r,
        fill="green"
    )

    #Trail points
    trail_points_ideal = []
    trail_points_linear_drag = []
    trail_points_quad = []

    t_ideal = [0.0]
    t_linear = [0.0]
    t_quad = [0.0]
    dt = 0.05

    phase = ["ascent"]       # linear drag phase
    phase_quad = ["moving"]  # quadratic drag phase

    def simulation():
        def update():

            #Ideal ball
            current_t_ideal = t_ideal[0]
            if current_t_ideal <= ideal_flight_time:
                x_ideal_m, y_ideal_m = ideal_position(current_t_ideal)
                cx_ideal = origin_x + x_ideal_m * pixels_x
                cy_ideal = origin_y - y_ideal_m * pixels_y
                canvas.coords(ideal_ball, cx_ideal - ball_r, cy_ideal - ball_r, cx_ideal + ball_r, cy_ideal + ball_r)
                trail_points_ideal.append((cx_ideal, cy_ideal))
                if len(trail_points_ideal) > 1:
                    x1, y1 = trail_points_ideal[-2]
                    x2, y2 = trail_points_ideal[-1]
                    canvas.create_line(x1, y1, x2, y2, fill="blue", width=2)
                t_ideal[0] += dt
            else:
                cx_ideal = origin_x + ideal_range * pixels_x
                cy_ideal = origin_y
                canvas.coords(ideal_ball, cx_ideal - ball_r, cy_ideal - ball_r, cx_ideal + ball_r, cy_ideal + ball_r)

            #Linear drag ball
            current_t_linear = t_linear[0]
            if phase[0] == "ascent":
                x_m = x_pos_linear_drag(current_t_linear)
                y_m = y_ascent_linear_drag(current_t_linear)
                if current_t_linear >= ascent_time_linear_drag:
                    phase[0] = "descent"
                    t_linear[0] = 0.0
                else:
                    cx = origin_x + x_m * pixels_x
                    cy = origin_y - y_m * pixels_y
                    canvas.coords(linear_ball, cx - ball_r, cy - ball_r, cx + ball_r, cy + ball_r)
                    trail_points_linear_drag.append((cx, cy))
                    if len(trail_points_linear_drag) > 1:
                        x1, y1 = trail_points_linear_drag[-2]
                        x2, y2 = trail_points_linear_drag[-1]
                        canvas.create_line(x1, y1, x2, y2, fill="red", width=2)
                    t_linear[0] += dt

            elif phase[0] == "descent":
                x_m = x_pos_linear_drag(ascent_time_linear_drag + current_t_linear)
                y_m = y_max_linear_drag - y_descent_linear_drag(current_t_linear)
                if y_m <= 0 or current_t_linear >= descent_time_linear_drag:
                    cx = origin_x + x_pos_linear_drag(t_total_linear_drag) * pixels_x
                    cy = origin_y
                    canvas.coords(linear_ball, cx - ball_r, cy - ball_r, cx + ball_r, cy + ball_r)
                    phase[0] = "done"
                else:
                    cx = origin_x + x_m * pixels_x
                    cy = origin_y - y_m * pixels_y
                    canvas.coords(linear_ball, cx - ball_r, cy - ball_r, cx + ball_r, cy + ball_r)
                    trail_points_linear_drag.append((cx, cy))
                    if len(trail_points_linear_drag) > 1:
                        x1, y1 = trail_points_linear_drag[-2]
                        x2, y2 = trail_points_linear_drag[-1]
                        canvas.create_line(x1, y1, x2, y2, fill="red", width=2)
                    t_linear[0] += dt

            #Quadratic drag ball
            current_t_quad = t_quad[0]
            if phase_quad[0] == "moving":
                y_q = y_pos_quad(current_t_quad)
                x_q = x_pos_quad(current_t_quad)
                if (y_q <= 0 and current_t_quad > 0.1) or current_t_quad >= t_total_quad:
                    cx = origin_x + x_range_quad * pixels_x
                    cy = origin_y
                    canvas.coords(quad_ball, cx - ball_r, cy - ball_r, cx + ball_r, cy + ball_r)
                    phase_quad[0] = "done"
                else:
                    cx = origin_x + x_q * pixels_x
                    cy = origin_y - y_q * pixels_y
                    canvas.coords(quad_ball, cx - ball_r, cy - ball_r, cx + ball_r, cy + ball_r)
                    trail_points_quad.append((cx, cy))
                    if len(trail_points_quad) > 1:
                        x1, y1 = trail_points_quad[-2]
                        x2, y2 = trail_points_quad[-1]
                        canvas.create_line(x1, y1, x2, y2, fill="green", width=2)
                    t_quad[0] += dt

            #Keep animation running until all three are done
            if t_ideal[0] <= ideal_flight_time or phase[0] != "done" or phase_quad[0] != "done":
                canvas.after(5, update)
                return

            else:
                def show_results():
                    results_canvas = tk.Canvas(main_window, width=W, height=H, bg="white")
                    results_canvas.place(x=0, y=0)

                    results_canvas.create_text(W / 2, H / 2 - 240, text="Simulation Results",
                                               font=("Arial", 24, "bold"), fill="black")

                    results_canvas.create_text(W / 2, H / 2 - 170, text="Ideal (No Drag)",
                                               font=("Arial", 18, "bold"), fill="blue")
                    results_canvas.create_text(W / 2, H / 2 - 130, text=f"Range: {ideal_range:.2f} m",
                                               font=("Arial", 16), fill="blue")
                    results_canvas.create_text(W / 2, H / 2 - 100, text=f"Max Height: {ideal_max_height:.2f} m",
                                               font=("Arial", 16), fill="blue")
                    results_canvas.create_text(W / 2, H / 2 - 70, text=f"Flight Time: {ideal_flight_time:.2f} s",
                                               font=("Arial", 16), fill="blue")

                    results_canvas.create_text(W / 2, H / 2, text="Linear Drag",
                                               font=("Arial", 18, "bold"), fill="red")
                    results_canvas.create_text(W / 2, H / 2 + 40, text=f"Range: {x_range_linear:.2f} m",
                                               font=("Arial", 16), fill="red")
                    results_canvas.create_text(W / 2, H / 2 + 70, text=f"Max Height: {y_max_linear_drag:.2f} m",
                                               font=("Arial", 16), fill="red")
                    results_canvas.create_text(W / 2, H / 2 + 100, text=f"Flight Time: {t_total_linear_drag:.2f} s",
                                               font=("Arial", 16), fill="red")

                    if angle_deg < 30:
                        quad_label = "Quadratic Drag (LAT)"
                    elif 30 <= angle_deg < 60:
                        quad_label = "Quadratic Drag (SAT)"
                    else:
                        quad_label = "Quadratic Drag (HAT)"

                    results_canvas.create_text(W / 2, H / 2 + 170, text=quad_label,
                                               font=("Arial", 18, "bold"), fill="green")
                    results_canvas.create_text(W / 2, H / 2 + 210, text=f"Range: {x_range_quad:.2f} m",
                                               font=("Arial", 16), fill="green")
                    results_canvas.create_text(W / 2, H / 2 + 240, text=f"Max Height: {y_max_quad:.2f} m",
                                               font=("Arial", 16), fill="green")
                    results_canvas.create_text(W / 2, H / 2 + 270, text=f"Flight Time: {t_total_quad:.2f} s",
                                               font=("Arial", 16), fill="green")

                    def go_back():
                        results_canvas.destroy()
                        back_button.destroy()

                    back_button = tk.Button(main_window, text="Back", bg="gray", fg="white", font=("Arial", 14),
                                            command=go_back)
                    back_button.place(x=W // 2 - 40, y=H / 2 + 320)

                results_button = tk.Button(main_window, text="Show Results", bg="green", fg="white",
                                           font=("Arial", 14), command=show_results)
                results_button.place(x=W - 200, y=40)

        update()

    run_simulation = tk.Button(
        main_window,
        text="Run the simulation!",
        bg="blue",
        fg="white",
        font=("Arial", 14),
        command=simulation
    )
    run_simulation.place(x=W - 200, y=0)


#Input
initial_velocity_question = tk.Label(main_window, text="Enter the initial velocity (m/s):", font=("Arial", 14))
initial_velocity_question.place(x=0, y=0)
initial_velocity_entry = tk.Entry(main_window, width=30, font=("Arial", 14))
initial_velocity_entry.place(x=0, y=30)

initial_angle_question = tk.Label(main_window, text="Enter the initial angle (degrees):", font=("Arial", 14))
initial_angle_question.place(x=0, y=70)
initial_angle_entry = tk.Entry(main_window, width=30, font=("Arial", 14))
initial_angle_entry.place(x=0, y=100)

initial_mass_question = tk.Label(main_window, text="Enter the mass (kg):", font=("Arial", 14))
initial_mass_question.place(x=0, y=140)
initial_mass_entry = tk.Entry(main_window, width=30, font=("Arial", 14))
initial_mass_entry.place(x=0, y=170)

enter_values = tk.Button(main_window,
                         text="Click to enter values!",
                         bg="blue", fg="white",
                         font=("Arial", 14),
                         command=enter_values_button)
enter_values.place(x=0, y=210)

main_window.mainloop()

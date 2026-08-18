
import pygame

def main():
    pygame.init()
    pygame.joystick.init()

    # Initialize the first joystick
    if pygame.joystick.get_count() == 0:
        print("No joystick detected. Please connect an Xbox controller.")
        return

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print(f"Controller Name: {joystick.get_name()}")
    print(f"Number of Axes: {joystick.get_numaxes()}")
    print(f"Number of Buttons: {joystick.get_numbuttons()}")
    print(f"Number of Hats: {joystick.get_numhats()}")

    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.JOYAXISMOTION:
                    # Print axis motion
                    for i in range(joystick.get_numaxes()):
                        axis_value = joystick.get_axis(i)
                        #print(f"Axis {i}: {axis_value:.2f}")

                elif event.type == pygame.JOYBUTTONDOWN:
                    # Print button press
                    print(f"Button {event.button} pressed")

                elif event.type == pygame.JOYBUTTONUP:
                    # Print button release
                    print(f"Button {event.button} released")

                elif event.type == pygame.JOYHATMOTION:
                    # Print hat (D-pad) motion
                    print(f"Hat {event.hat}: {event.value}")

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        pygame.quit()

if __name__ == '__main__':
    main()
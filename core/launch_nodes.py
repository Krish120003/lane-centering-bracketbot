
import libtmux
import sys
import os

def create_nodes_environment():
    # Create a new server instance
    server = libtmux.Server()

    # Define session name
    SESSION_NAME = "nodes-environment"

    # Kill existing session if it exists
    try:
        existing_session = server.get_session(SESSION_NAME)
        if existing_session:
            existing_session.kill()
    except:
        pass

    try:
        # Start a new session
        session = server.new_session(
            session_name=SESSION_NAME,
            window_name="nodes",
            detach=True
        )

        # Get the first window and pane
        main_window = session.windows[0]
        main_pane = main_window.attached_pane

        # Create the 2x3 grid
        # First row splits
        right_pane1 = main_pane.split_window(vertical=True)
        right_pane2 = right_pane1.split_window(vertical=True)
        
        # Second row splits
        bottom_left_pane = main_pane.split_window()
        bottom_middle_pane = right_pane1.split_window()
        bottom_right_pane = right_pane2.split_window()

        # Ensure even layout
        main_window.select_layout('tiled')

        # Send commands to each pane
        main_pane.send_keys('cd ~/quickstart/core && python3 node_drive.py')
        right_pane1.send_keys('cd ~/quickstart/core && python3 node_map.py')
        right_pane2.send_keys('cd ~/quickstart/core && python3 node_odometry.py')
        bottom_left_pane.send_keys('cd ~/quickstart/core && python3 node_pathplanning.py')
        bottom_middle_pane.send_keys('cd ~/quickstart/core && python3 node_rerun.py')
        bottom_right_pane.send_keys('cd ~/quickstart/core')  # Empty terminal

        print(f"Tmux session '{SESSION_NAME}' created successfully!")
        
        # Attach to session if not already in tmux
        if not os.environ.get('TMUX'):
            os.system(f"tmux attach -t {SESSION_NAME}")
        else:
            print(f"To attach to this session later, use: tmux attach -t {SESSION_NAME}")

    except Exception as e:
        print(f"Error creating tmux session: {e}")
        sys.exit(1)

if __name__ == "__main__":
    create_nodes_environment()
import threading
from smart_car_py_pkg.cart_gui import SmartCartServer
from smart_car_py_pkg.gui.robot_gui import main as robot_main

def run_flask():
    server = SmartCartServer()
    server.app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)

def main(args=None):
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()

    robot_main(args)

if __name__ == '__main__':
    main()

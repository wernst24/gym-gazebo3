#!/usr/bin/env python3

import argparse
import os
import subprocess
import webbrowser
import time


def main():
    parser = argparse.ArgumentParser(description='Visualize training progress with Tensorboard')
    parser.add_argument('--logdir', type=str, default='./logs/',
                       help='Tensorboard log directory')
    parser.add_argument('--port', type=int, default=6006,
                       help='Port for Tensorboard server')
    parser.add_argument('--host', type=str, default='localhost',
                       help='Host for Tensorboard server')
    parser.add_argument('--open-browser', action='store_true',
                       help='Automatically open browser')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.logdir):
        print(f"Log directory {args.logdir} does not exist!")
        return
    
    # Start Tensorboard
    print(f"Starting Tensorboard on {args.host}:{args.port}")
    print(f"Log directory: {args.logdir}")
    
    cmd = [
        'tensorboard',
        '--logdir', args.logdir,
        '--host', args.host,
        '--port', str(args.port)
    ]
    
    try:
        # Start Tensorboard process
        process = subprocess.Popen(cmd)
        
        # Wait a bit for server to start
        time.sleep(3)
        
        # Open browser if requested
        if args.open_browser:
            url = f"http://{args.host}:{args.port}"
            print(f"Opening browser at {url}")
            webbrowser.open(url)
        
        print("Tensorboard is running. Press Ctrl+C to stop.")
        
        # Wait for process to finish
        process.wait()
        
    except KeyboardInterrupt:
        print("\nStopping Tensorboard...")
        process.terminate()
        process.wait()
        
    except Exception as e:
        print(f"Error starting Tensorboard: {e}")


if __name__ == '__main__':
    main()
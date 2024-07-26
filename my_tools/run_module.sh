#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <module_name>"
    exit 1
fi

module_name=$1

source cyber/setup.bash 

case $module_name in
    control)
        echo "Run localization module..."
        mainboard -d modules/control/control_component/dag/control.dag
        ;;
    planning)
        echo "Run planning module..."
        mainboard -d /apollo/modules/planning/planning_component/dag/planning.dag -s CYBER_DEFAULT
        ;;
    canbus)
        echo "Run canbus module..."
        mainboard -d modules/canbus/dag/canbus.dag 
        ;;
    routing)
        echo "Run request routing module..."
        python my_tools/cyber_routing.py 
        ;;
    ins)
    echo "Run localization module..."
    ./bazel-bin/modules/ap_localization/ins_interface_main 
    ;;
    *)
        echo "Error: Unknown module '$module_name'"
        exit 2
        ;;
esac

echo "Operation completed."
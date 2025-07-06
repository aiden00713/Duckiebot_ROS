#!/bin/bash

# 啟動 ROS 環境
source /environment.sh
dt-launchfile-init

# 定義 Duckiebot 的 ROS 節點
declare -A NODES=(
    ["1"]="camera-node_straight"
    ["2"]="camera-node_turn"
    ["3"]="wheel-node_control"
    ["4"]="wheel-node_control-PID"
    ["5"]="control"
    ["8"]="wheel_node-test"
    ["9"]="auto-screenshot"
)

# 定義要監控的 ROS 主題及其描述
declare -A TOPICS=(
    #["/duckiebot06/camera_node_straight/angles"]="🔹 直線模式偵測到的車道角度"
    #["/duckiebot06/camera_node_straight/offset"]="🔹 直線模式的車道偏移量"
    #["/duckiebot06/camera_node_turn/angles"]="🔄 轉彎模式偵測到的轉彎角度"
    #["/duckiebot06/camera_node_turn/inter_dist"]="📏 距離前方轉彎區域的距離"
    #["/duckiebot06/wheel_control_node/command"]="⚙️  馬達控制指令"
    #["/duckiebot06/front_center_tof_driver_node/range"]="🚧 ToF"
)

# 清除舊的 rostopic 訂閱數據
rm -f /tmp/rostopic_monitor_*.log

# **持續監聽 rostopic**
function monitor_topics() {
    # 停止所有舊的 rostopic 監聽，避免多次疊加
    pkill -f "rostopic echo"

    for topic in "${!TOPICS[@]}"; do
        log_file="/tmp/rostopic_monitor_${topic//\//_}.log"

        # 確保 topic 存在才監聽，否則顯示未發佈
        if ! rostopic list | grep -q "$topic"; then
            echo "⚠️ 未發佈" > "$log_file"
            continue
        fi
        
        # **處理不同類型的 rostopic 訂閱數據**
        if [[ "$topic" == "/duckiebot06/front_center_tof_driver_node/range" ]]; then
            { stdbuf -oL rostopic echo "$topic" | awk '$1 ~ /^range:$/ {print int($2 * 1000)}' > "$log_file" & } &
        else
            { stdbuf -oL rostopic echo "$topic" | awk '/data:/ {print $2}' > "$log_file" & } &
        fi
    done
}

# **顯示選單**
function show_menu() {
    clear
    echo "🚗 Duckiebot ROS 訂閱數據監控 🚀"
    echo "---------------------------------"
    echo "📝 目前 rostopic 訂閱數據："

    for topic in "${!TOPICS[@]}"; do
        log_file="/tmp/rostopic_monitor_${topic//\//_}.log"
        if [[ -f "$log_file" ]]; then
            latest_value=$(tail -n 1 "$log_file" | tr -d '\n')  # 讀取最新數據
            if [[ -z "$latest_value" ]]; then
                latest_value="⚠️ 無數據"
            fi
            echo "${TOPICS[$topic]}: ${latest_value}"
        else
            echo "${TOPICS[$topic]}: ⚠️ 未發佈"
        fi
    done

    echo "---------------------------------"
    echo "👉 輸入1234啟動所有節點，或輸入單個數字選擇:"
    for key in "${!NODES[@]}"; do
        echo "[$key] 啟動 ${NODES[$key]}"
    done
    echo "[X] 停止所有節點"
    echo "[Q] 退出"
    echo "[C] 保留 straight.py 關閉其他節點"

}

# **啟動指定的 ROS 節點**
function start_node() {
    node_name=$1
    echo "🔄 啟動 $node_name..."
    rosrun my_package "$node_name.py" &
    PID=$!
    echo "$node_name $PID" >> /tmp/ros_nodes.pid
}

# **停止所有 ROS 節點**
function stop_all_nodes() {
    echo "🛑 停止所有節點..."
    pkill -f "rostopic echo"  # 停止 rostopic 監聽
    if [ -f /tmp/ros_nodes.pid ]; then
        while read -r line; do
            kill "$(echo $line | awk '{print $2}')" 2>/dev/null
        done < /tmp/ros_nodes.pid
        rm -f /tmp/ros_nodes.pid
    fi
    echo "✅ 所有節點已停止"
}

# **解析用戶輸入並啟動對應的節點**
function process_user_input() {
    input_str=$1
    for ((i=0; i<${#input_str}; i++)); do
        key="${input_str:$i:1}"  # 取出單個字元
        if [[ "${NODES[$key]}" ]]; then
            start_node "${NODES[$key]}"
            sleep 3  # 確保節點啟動
            monitor_topics  # 重新啟動 `rostopic` 監聽
        else
            echo "❌ 無效選擇: $key"
        fi
    done
}


# **保留 camera-node_straight.py，關閉其他 ROS 節點**
function stop_except_camera_node_straight() {
    keep_node="camera-node_straight.py"
    echo "🔄 關閉除了 $keep_node 的其他節點..."

    if [ -f /tmp/ros_nodes.pid ]; then
        while read -r line; do
            node_name=$(echo "$line" | awk '{print $1}')
            pid=$(echo "$line" | awk '{print $2}')
            if [[ "$node_name" != "$keep_node" ]]; then
                echo "🛑 關閉節點 $node_name (PID $pid)"
                kill "$pid" 2>/dev/null
            else
                echo "✅ 保留節點 $node_name"
            fi
        done < /tmp/ros_nodes.pid
        grep "$keep_node" /tmp/ros_nodes.pid > /tmp/ros_nodes_new.pid
        mv /tmp/ros_nodes_new.pid /tmp/ros_nodes.pid
    fi

    pkill -f "rostopic echo"
    monitor_topics  # 重新監聽
}


# **確保 `rostopic` 監聽在節點啟動後運行**
monitor_topics &

# **進入互動模式**
while true; do
    show_menu
    echo -n "請輸入選擇: "
    read -r choice

    case $choice in
        [1-9]*)
            process_user_input "$choice"
            ;;
        X|x)
            stop_all_nodes
            ;;
        Q|q)
            stop_all_nodes
            echo "👋 退出 Duckiebot 控制系統"
            exit 0
            ;;
        C|c)
            stop_except_camera_node_straight
            ;;

        *)
            echo "❌ 無效選擇，請重新輸入"
            ;;
    esac
    sleep 2  # 每 2 秒更新一次數據，避免刷屏
done

# 2025.07.06
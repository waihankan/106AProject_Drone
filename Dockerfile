FROM --platform=linux/amd64 osrf/ros:humble-desktop

# Install terminal tools and dependencies
RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    mesa-utils \
    libglapi-mesa \
    libosmesa6 \
    libx11-xcb1 \
    libxcb-render0 \
    libxcb-shm0 \
    libxcb-xfixes0 \
    libxcb-glx0 \
    fzf \
    ripgrep \
    bat \
    exa \
    curl \
    git \
    vim \
    tmux \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Set up automatic ROS sourcing and terminal improvements
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash' >> /root/.bashrc && \
    echo 'export HISTSIZE=10000' >> /root/.bashrc && \
    echo 'export HISTFILESIZE=20000' >> /root/.bashrc && \
    echo 'export HISTCONTROL=ignoredups:erasedups' >> /root/.bashrc && \
    echo 'shopt -s histappend' >> /root/.bashrc && \
    echo 'PROMPT_COMMAND="history -a; history -c; history -r; $PROMPT_COMMAND"' >> /root/.bashrc && \
    echo 'alias ll="exa -la"' >> /root/.bashrc && \
    echo 'alias ls="exa"' >> /root/.bashrc && \
    echo 'alias cat="batcat"' >> /root/.bashrc && \
    echo 'export FZF_DEFAULT_COMMAND="rg --files --hidden --glob \"!.git\""' >> /root/.bashrc && \
    echo 'export FZF_CTRL_T_COMMAND="$FZF_DEFAULT_COMMAND"' >> /root/.bashrc && \
    echo '# ROS2 aliases' >> /root/.bashrc && \
    echo 'alias cb="colcon build"' >> /root/.bashrc && \
    echo 'alias cbs="colcon build --symlink-install"' >> /root/.bashrc && \
    echo 'alias cbp="colcon build --packages-select"' >> /root/.bashrc && \
    echo 'alias ct="colcon test"' >> /root/.bashrc && \
    echo 'alias ctp="colcon test --packages-select"' >> /root/.bashrc && \
    echo 'alias ctr="colcon test-result --verbose"' >> /root/.bashrc && \
    echo 'alias ss="source install/setup.bash"' >> /root/.bashrc && \
    echo 'alias rl="ros2 launch"' >> /root/.bashrc && \
    echo 'alias rr="ros2 run"' >> /root/.bashrc && \
    echo 'alias rt="ros2 topic"' >> /root/.bashrc && \
    echo 'alias rn="ros2 node"' >> /root/.bashrc && \
    echo 'alias rs="ros2 service"' >> /root/.bashrc && \
    echo 'alias rp="ros2 param"' >> /root/.bashrc && \
    echo 'alias ri="ros2 interface"' >> /root/.bashrc && \
    echo 'alias rb="ros2 bag"' >> /root/.bashrc && \
    echo 'alias rtl="ros2 topic list"' >> /root/.bashrc && \
    echo 'alias rte="ros2 topic echo"' >> /root/.bashrc && \
    echo 'alias rti="ros2 topic info"' >> /root/.bashrc && \
    echo 'alias rth="ros2 topic hz"' >> /root/.bashrc && \
    echo 'alias rnl="ros2 node list"' >> /root/.bashrc && \
    echo 'alias rni="ros2 node info"' >> /root/.bashrc && \
    echo 'alias rsl="ros2 service list"' >> /root/.bashrc && \
    echo 'alias rsi="ros2 service type"' >> /root/.bashrc && \
    echo 'alias rsc="ros2 service call"' >> /root/.bashrc && \
    echo 'alias rpl="ros2 param list"' >> /root/.bashrc && \
    echo 'alias rpg="ros2 param get"' >> /root/.bashrc && \
    echo 'alias rps="ros2 param set"' >> /root/.bashrc && \
    echo 'alias rviz="ros2 run rviz2 rviz2"' >> /root/.bashrc && \
    echo 'alias rqt="ros2 run rqt_gui rqt_gui"' >> /root/.bashrc && \
    echo '# Utility aliases' >> /root/.bashrc && \
    echo 'alias ..="cd .."' >> /root/.bashrc && \
    echo 'alias ...="cd ../.."' >> /root/.bashrc && \
    echo 'alias ....="cd ../../.."' >> /root/.bashrc && \
    echo 'alias grep="grep --color=auto"' >> /root/.bashrc && \
    echo 'alias fgrep="fgrep --color=auto"' >> /root/.bashrc && \
    echo 'alias egrep="egrep --color=auto"' >> /root/.bashrc && \
    echo 'alias tree="exa --tree"' >> /root/.bashrc && \
    echo 'alias la="exa -la"' >> /root/.bashrc && \
    echo 'alias l="exa -l"' >> /root/.bashrc && \
    echo 'alias df="df -h"' >> /root/.bashrc && \
    echo 'alias du="du -h"' >> /root/.bashrc && \
    echo 'alias free="free -h"' >> /root/.bashrc && \
    echo 'alias ps="ps aux"' >> /root/.bashrc

# Install TPM (Tmux Plugin Manager) and set up tmux config
RUN git clone https://github.com/tmux-plugins/tpm ~/.tmux/plugins/tpm

# Create tmux configuration
RUN cat > /root/.tmux.conf << 'EOF'
# ~/.tmux.conf

# -----------------------------
# 1. PREFIX KEY: Backtick (`) 
# -----------------------------
unbind C-b
set -g prefix `
bind ` send-prefix

# Optional: Reload config with ` + r
bind r source-file ~/.tmux.conf \; display "✅ Config reloaded"


# -----------------------------
# 2. MOUSE SUPPORT (ON)
# -----------------------------
set -g mouse on
set -g default-terminal "tmux-256color"


# -----------------------------
# 3. START INDEX FROM 1
# -----------------------------
set -g base-index 1
set -g pane-base-index 1
setw -g pane-base-index 1
set -g renumber-windows on


# -----------------------------
# 4. PANE & WINDOW NAVIGATION
# -----------------------------

# New window
bind n new-window -c "#{pane_current_path}"

# Split panes
bind - split-window -h -c "#{pane_current_path}"  # ` + - → horizontal
bind _ split-window -v -c "#{pane_current_path}"  # ` + _ → vertical

# Unbind defaults
unbind '"'
unbind %

# Vim-style pane selection (with prefix)
bind h select-pane -L
bind j select-pane -D
bind k select-pane -U
bind l select-pane -R

# Switch panes WITHOUT prefix: Alt + Arrow (optional)
bind -n M-Left  select-pane -L
bind -n M-Right select-pane -R
bind -n M-Up    select-pane -U
bind -n M-Down  select-pane -D

# Switch windows: Shift + Arrow
bind -n S-Left  previous-window
bind -n S-Right next-window

# Switch windows: Alt + H/L
bind -n M-H previous-window
bind -n M-L next-window


# -----------------------------
# 5. VI MODE & COPY SELECTION
# -----------------------------
setw -g mode-keys vi

# Enter copy mode (useful for large selections)
bind-key -T vi-edit Escape copy-mode

# In copy mode: use vi-like selections
bind-key -T copy-mode-vi v         send-keys -X begin-selection
bind-key -T copy-mode-vi C-v      send-keys -X rectangle-toggle
bind-key -T copy-mode-vi y        send-keys -X copy-selection-and-cancel
bind-key -T copy-mode-vi Enter    send-keys -X copy-selection-and-cancel

# Allow mouse drag to select and copy (like Terminal.app)
# This works because mouse=on + no override
bind-key -T copy-mode-vi MouseDragEnd1Pane send-keys -X copy-selection-and-cancel

set -g status-position bottom
set -g status-bg colour234
set -g status-fg colour137
set -g status-left ''
set -g status-right '#[fg=colour233,bg=colour241,bold] %d/%m #[fg=colour233,bg=colour245,bold] %H:%M:%S '
set -g status-right-length 80
set -g status-left-length 20
setw -g window-status-current-format ' #I#[fg=colour250]:#[fg=colour255]#W#[fg=colour50]#F '
setw -g window-status-format ' #I#[fg=colour237]:#[fg=colour250]#W#[fg=colour244]#F '


# -----------------------------
# 6. PLUGINS (TPM)
# -----------------------------

# load plugins first
set -g @plugin 'tmux-plugins/tpm'
set -g @plugin 'tmux-plugins/tmux-sensible'
set -g @plugin 'christoomey/vim-tmux-navigator'
set -g @plugin 'tmux-plugins/tmux-yank'
set -g @plugin 'tmux-plugins/tmux-resurrect'

# This must be at the very end!
run '~/.tmux/plugins/tpm/tpm'
EOF

# Install tmux plugins (this will run TPM install)
RUN ~/.tmux/plugins/tpm/scripts/install_plugins.sh

# Copy and install Python dependencies for Tello
COPY ros_workspace/requirements.txt /tmp/requirements.txt
RUN pip3 install -r /tmp/requirements.txt && rm /tmp/requirements.txt

# Setup workspace initialization script
RUN echo '# Auto-source ROS2 workspace if it exists' >> /root/.bashrc && \
    echo 'if [ -f /root/ros_workspace/install/setup.bash ]; then' >> /root/.bashrc && \
    echo '    source /root/ros_workspace/install/setup.bash' >> /root/.bashrc && \
    echo 'fi' >> /root/.bashrc

# Create COLCON_IGNORE for SLAM packages that require OpenCV 3
RUN mkdir -p /root/ros_workspace/src && \
    echo '# This directory is ignored by colcon build' > /root/.colcon_ignore_setup.sh && \
    echo 'if [ -d "/root/ros_workspace/src/tello-ros2/slam" ]; then' >> /root/.colcon_ignore_setup.sh && \
    echo '    touch /root/ros_workspace/src/tello-ros2/slam/COLCON_IGNORE' >> /root/.colcon_ignore_setup.sh && \
    echo 'fi' >> /root/.colcon_ignore_setup.sh && \
    chmod +x /root/.colcon_ignore_setup.sh

# Set working directory
WORKDIR /root/ros_workspace

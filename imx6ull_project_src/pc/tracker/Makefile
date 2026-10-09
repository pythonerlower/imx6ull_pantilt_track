# PC 侧没有编译步骤，这个 Makefile 只是把测试入口统一起来，
# 让根 Makefile 与 scripts/build_all.sh 能一致地调用。
PYTHON ?= python3

.PHONY: all test test-unit test-plane clean

all:
	@echo "PC 侧为纯 Python，无需编译。测试用 make test。"

# 纯本地单测：不出网、不起外部进程。随时可跑。
# 只有 test_video_plane.py 会开回环 UDP 口（9299）收发——那正是它要验的东西
# （收帧线程、残帧过滤、取帧落后时的丢弃策略），用假 socket 替掉就什么都验不到了。
# 它占约 8 秒，其余几个加起来不到 4 秒。
test-unit:
	$(PYTHON) -m pytest -q . 2>/dev/null || { \
		$(PYTHON) test_video_rx.py && \
		$(PYTHON) test_control.py && \
		$(PYTHON) test_tracker.py && \
		$(PYTHON) test_display.py && \
		$(PYTHON) test_video_plane.py && \
		$(PYTHON) test_auto.py && \
		$(PYTHON) test_patrol.py && \
		$(PYTHON) selftest_vectors.py --check; }

# 端到端联调：需要先编出宿主版 daemon（见 user/pantilt_daemon/Makefile）。
# 路径是 `../../`：pc/ 与 user/ 现在是 imx6ull_project_src/ 下的兄弟
# （2026-10-07 那次"程序源文件目录整改"漏改了这里，一直指不到）。
DAEMON ?= ../../user/pantilt_daemon/pantilt_daemon_host

test-plane: $(DAEMON)
	$(PYTHON) test_plane.py --daemon $(DAEMON)

test: test-unit
	@if [ -x "$(DAEMON)" ]; then \
		$(PYTHON) test_plane.py --daemon $(DAEMON); \
	else \
		echo "[skip] 未找到宿主版 daemon（$(DAEMON)），跳过端到端联调"; \
		echo "       先跑：make -C ../../user/pantilt_daemon test"; \
	fi

clean:
	$(RM) -r __pycache__

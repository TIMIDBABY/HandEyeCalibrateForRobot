import socket
import time

"""
用以测试UR5的网络连接
"""

robot_ip = "192.168.1.105"  # 修改为你的机器人IP


def test_ur_connection(ip_address, ports_to_test=None):
    """测试UR机器人的网络连接"""
    if ports_to_test is None:
        ports_to_test = [30001, 30002, 30003, 30004]

    print(f"测试连接到 UR 机器人: {ip_address}")
    print("=" * 50)

    # 1. 基本ping测试（如果需要，可以使用subprocess调用ping）
    print("1. 网络连通性测试:")
    try:
        # 简单的socket连接测试代替ping
        test_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        test_socket.settimeout(2)
        result = test_socket.connect_ex((ip_address, 80))  # 测试任意端口
        test_socket.close()
        if result == 0:
            print(f"   ✓ 网络可达: {ip_address}")
        else:
            print(f"   ✗ 网络不可达: {ip_address}")
    except Exception as e:
        print(f"   ✗ 网络测试失败: {e}")

    # 2. 端口连接测试
    print("\n2. UR端口连接测试:")
    successful_ports = []

    for port in ports_to_test:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(3)  # 3秒超时
            result = sock.connect_ex((ip_address, port))

            if result == 0:
                print(f"   ✓ 端口 {port}: 连接成功")
                successful_ports.append(port)

                # 尝试接收一些数据来验证是否为UR接口
                try:
                    sock.settimeout(1)
                    data = sock.recv(100)
                    if data:
                        print(f"      数据长度: {len(data)} 字节")
                except socket.timeout:
                    print("      (未在1秒内收到数据)")
                except Exception:
                    pass
            else:
                print(f"   ✗ 端口 {port}: 连接失败 (错误代码: {result})")

            sock.close()

        except Exception as e:
            print(f"   ✗ 端口 {port}: 异常 - {e}")

        time.sleep(0.1)  # 短暂延迟避免过快连接

    # 3. 总结和建议
    print(f"\n3. 连接测试总结:")
    if successful_ports:
        print(f"   成功连接的端口: {successful_ports}")
        if 30003 in successful_ports:
            print("   ✓ 端口30003可用，你的原始代码应该能工作")
        else:
            print(f"   建议尝试使用端口: {successful_ports[0]}")
            print("   修改代码中的 tcp_port 参数")
    else:
        print("   ✗ 所有端口连接失败")
        print("   请检查：")
        print("     - 机器人IP地址是否正确")
        print("     - 机器人是否已开机并连接网络")
        print("     - 防火墙设置")
        print("     - 网络配置")


def test_specific_ur_interface(ip_address, port=30003):
    """测试特定UR接口并尝试获取机器人状态"""
    print(f"\n4. 详细接口测试 (端口 {port}):")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)

        print(f"   连接到 {ip_address}:{port}...")
        sock.connect((ip_address, port))
        print("   ✓ 连接成功")

        # 尝试接收UR状态数据
        print("   尝试接收状态数据...")
        data = sock.recv(1500)

        if data and len(data) > 100:
            print(f"   ✓ 接收到状态数据: {len(data)} 字节")
            print("   这看起来是有效的UR接口")

            # 简单解析消息大小（前4字节）
            import struct

            if len(data) >= 4:
                msg_size = struct.unpack("!i", data[0:4])[0]
                print(f"   消息大小: {msg_size}")
        else:
            print(f"   接收到数据: {len(data) if data else 0} 字节")
            print("   数据可能不完整或接口类型不正确")

        sock.close()
        return True

    except socket.timeout:
        print("   ✗ 连接超时")
        return False
    except ConnectionRefusedError:
        print("   ✗ 连接被拒绝")
        return False
    except Exception as e:
        print(f"   ✗ 连接失败: {e}")
        return False


if __name__ == "__main__":

    # 运行连接测试
    test_ur_connection(robot_ip)

    # 如果端口30003可用，进行详细测试
    print("\n" + "=" * 50)
    test_specific_ur_interface(robot_ip, 30003)

    print("\n" + "=" * 50)
    print("测试完成!")
    print(
        "\n🎯 Tips：如果计算机与机器人不在同一网段，则测试一会不通过，只要测试二三四通过即可"
    )
    print("\n故障排除建议:")
    print("1. 如果所有端口都连接失败：")
    print("   - 检查机器人IP地址 (在示教器 Settings > System > Network)")
    print("   - 确保机器人已连接到网络")
    print("   - 检查电脑和机器人是否在同一网段")
    print("2. 如果只有部分端口可用：")
    print("   - 使用可用的端口号修改你的代码")
    print("3. 如果连接成功但无数据：")
    print("   - 机器人可能处于保护停止状态")
    print("   - 检查机器人是否有错误或警告")

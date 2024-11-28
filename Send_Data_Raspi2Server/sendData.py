import paramiko
import os
import time

# SSH and SCP configurations
ssh_host = '172.16.12.74'  # Replace with your server's IP address
ssh_user = 'alzob'  # Replace with your server username
ssh_password = '@Samrat7778'  # Replace with your server password

# Local and remote directory paths
local_dir = '/home/zobaerpi/catkin_ws/src/test/src/picture'
remote_dir = 'E:/RaspberryPi'

def transfer_files(ssh, sftp, local_dir, remote_dir):
    for filename in os.listdir(local_dir):
        local_file = os.path.join(local_dir, filename)
        remote_file = os.path.join(remote_dir, filename)
        sftp.put(local_file, remote_file)
        print(f"Transferred {local_file} to {remote_file}")

def main():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(ssh_host, username=ssh_user, password=ssh_password)

    sftp = ssh.open_sftp()
    transfer_files(ssh, sftp, local_dir, remote_dir)

    sftp.close()
    ssh.close()

if __name__ == '__main__':
    while True:
        main()
        break
        # time.sleep(60)  # Adjust the interval as needed
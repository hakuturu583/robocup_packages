#!/usr/bin/env python
import pexpect
import rospy

USER = "nao"
PASSWORD = "nao"

if __name__ == "__main__":
    rospy.init_node('gc_bridge_runner', anonymous=False)
    team_number = rospy.get_param(rospy.get_name()+'/team_number')
    player_number = rospy.get_param(rospy.get_name()+'/player_number')
    nao_ip = rospy.get_param(rospy.get_name()+'/nao_ip')
    remote_shell = pexpect.spawn('ssh %s@%s' % (USER, nao_ip))
    remote_shell.expect('.*ssword:')
    remote_shell.sendline(PASSWORD)
    remote_shell.sendline("cd lib")
    cmd = "./gcbridge " + str(team_number) + " " + str(player_number)
    remote_shell.sendline(cmd)
    #remote_shell.interact()
    rospy.spin()

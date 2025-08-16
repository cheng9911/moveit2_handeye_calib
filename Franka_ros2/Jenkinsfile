pipeline {
    agent {
        dockerfile true 
    }
    triggers {
        pollSCM('H/5 * * * *')
    }
    stages {
        stage('Init') {
            steps {
                script {
                    notifyBitbucket()
                }
                sh 'rm -rf build log install'
                sh 'vcs import < franka.repos --recursive'
            }
        }
        stage('Build') {
            steps {
                sh '''
                    . /opt/ros/humble/setup.sh
                    colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCHECK_TIDY=ON -DBUILD_TESTS=OFF
                '''
            }
        }
        stage('Test') {
            steps {
                sh '''
                    . /opt/ros/humble/setup.sh
                    . install/setup.sh
                    colcon test --packages-ignore libfranka --event-handlers console_direct+
                    colcon test-result --verbose
                '''
            }
            post {
                always {
                    junit 'build/**/test_results/**/*.xml'
                }
            }
        }
    }
    post {
        always {
            cleanWs()
            script {
                notifyBitbucket()
            }
        }
    }
}

pipeline {
    agent any

    options {
        disableConcurrentBuilds()
    }

    stages {
        stage('Build') {
            steps {
                ansiColor('xterm') {
                    sh 'make'
                }
            }
        }

        stage('Test') {
            steps {
                ansiColor('xterm') {
                    sh 'prosv5 lsusb'
                    sh 'prosv5 ut'
                }
            }
        }
    }

    post {
        always {
            ansiColor('xterm') {
                sh 'prosv5 v5 stop'
            }
        }
    }
}

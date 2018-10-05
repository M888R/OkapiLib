pipeline {
    agent any

    options {
        disableConcurrentBuilds()
    }

    stages {
        stage('Build') {
            steps {
                sh 'make'
                sh 'prosv5 lsusb'
            }
        }
    }
}

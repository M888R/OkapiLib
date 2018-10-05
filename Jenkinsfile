pipeline {
    agent any

    stages {
        stage('Build') {
            steps {
              dir('OkapiLib') {
                sh('./Makefile')
                }
            }
        }
    }
}

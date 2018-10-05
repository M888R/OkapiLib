pipeline {
    agent any

    stages {
        stage('Build') {
          steps{
            checkout scm
            sh 'make'
        
            cleanWs()
            }
        }

    }
}

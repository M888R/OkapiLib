pipeline {
    agent any

    stages {
        stage('Build') {
          node{
            checkout scm
            sh 'mvn clean install'
            sh 'make'
            }
        }

    }
}
